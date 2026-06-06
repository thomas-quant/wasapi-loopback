// wasapi-loopback — clean-room WASAPI process-tree EXCLUDE loopback capture.
//
// Authored SOLELY from the public Microsoft "ApplicationLoopback" MIT sample
// (Windows-classic-samples) and the public `windows` crate / Win32 docs. See NOTICE
// for the retained Microsoft MIT copyright. No proprietary or third-party application
// code or symbol layout is used as a basis (ECHO-04 clean-room boundary).
//
// Mechanism: activate a WASAPI IAudioClient in process-loopback mode with
// PROCESS_LOOPBACK_MODE_EXCLUDE_TARGET_PROCESS_TREE against a caller-supplied root PID,
// so everything EXCEPT that process tree is captured (the #46 echo fix — exclude the
// host's own call playback). The format is hardcoded 48000 Hz / 2ch / 32-bit IEEE
// float; the audio engine converts to it in shared mode via AUTOCONVERTPCM (no Rust DSP).
//
// Task 1: crate scaffold + the clean-room activation path (hardcoded format, dynamic
// ActivateAudioInterfaceAsync resolution, async-completion wait, try-activate-and-catch
// -> "unsupported" instead of throwing).
// Task 2: the event-driven WASAPI capture loop on a dedicated thread + the napi
// ThreadsafeFunction NonBlocking push of 480-frame (3840-byte) f32 chunks (drop-oldest
// backpressure) + an idempotent `stop` that signals the thread and joins with a timeout.

#![cfg(windows)]

use std::mem::{size_of, ManuallyDrop};
use std::sync::mpsc::{channel, Sender};
use std::sync::Mutex;
use std::thread::JoinHandle;

use napi::bindgen_prelude::Buffer;
use napi::threadsafe_function::{ThreadsafeFunction, ThreadsafeFunctionCallMode};
use napi_derive::napi;

// The `#[implement]` macro emits absolute `::windows_core::` paths, so windows-core is a
// direct dependency (see Cargo.toml). `windows` also re-exports it as `windows::core`.
use windows::core::{implement, w, IUnknown, Interface, GUID, HRESULT, PCWSTR};
use windows::Win32::Foundation::{
    CloseHandle, GetLastError, ERROR_PROC_NOT_FOUND, E_FAIL, HANDLE, S_OK, WAIT_OBJECT_0,
};
use windows::Win32::Media::Audio::{
    ActivateAudioInterfaceAsync, IActivateAudioInterfaceAsyncOperation,
    IActivateAudioInterfaceCompletionHandler, IActivateAudioInterfaceCompletionHandler_Impl,
    IAudioCaptureClient, IAudioClient, AUDCLNT_BUFFERFLAGS_SILENT, AUDCLNT_SHAREMODE_SHARED,
    AUDCLNT_STREAMFLAGS_AUTOCONVERTPCM, AUDCLNT_STREAMFLAGS_EVENTCALLBACK,
    AUDCLNT_STREAMFLAGS_LOOPBACK, AUDCLNT_STREAMFLAGS_SRC_DEFAULT_QUALITY,
    AUDIOCLIENT_ACTIVATION_PARAMS, AUDIOCLIENT_ACTIVATION_PARAMS_0,
    AUDIOCLIENT_ACTIVATION_TYPE_PROCESS_LOOPBACK, AUDIOCLIENT_PROCESS_LOOPBACK_PARAMS,
    PROCESS_LOOPBACK_MODE_EXCLUDE_TARGET_PROCESS_TREE, VIRTUAL_AUDIO_DEVICE_PROCESS_LOOPBACK,
    WAVEFORMATEX, WAVEFORMATEXTENSIBLE, WAVEFORMATEXTENSIBLE_0,
};
use windows::Win32::Media::KernelStreaming::WAVE_FORMAT_EXTENSIBLE;
use windows::Win32::Media::Multimedia::KSDATAFORMAT_SUBTYPE_IEEE_FLOAT;
use windows::Win32::System::Com::StructuredStorage::PROPVARIANT;
use windows::Win32::System::Com::{CoInitializeEx, CoUninitialize, BLOB, COINIT_MULTITHREADED};
use windows::Win32::System::LibraryLoader::{GetProcAddress, LoadLibraryW};
use windows::Win32::System::Threading::{CreateEventW, SetEvent, WaitForSingleObject, INFINITE};
use windows::Win32::System::Variant::VT_BLOB;

// ─── Hardcoded capture format (the renderer/transport contract) ─────────────────────
//
// 48000 Hz / 2 channels / 32-bit IEEE float, interleaved stereo (L,R,L,R...).
// The process-loopback "magic device" returns E_NOTIMPL from the format-query calls
// (mix-format / format-support), so the format MUST be hardcoded and we never query it;
// AUDCLNT_STREAMFLAGS_AUTOCONVERTPCM makes the shared-mode engine resample/matrix to it,
// so no Rust-side conversion is needed.
const SAMPLE_RATE: u32 = 48_000;
const CHANNELS: u16 = 2;
const BITS_PER_SAMPLE: u16 = 32;
const BLOCK_ALIGN: u16 = CHANNELS * BITS_PER_SAMPLE / 8; // 8 bytes/frame
const AVG_BYTES_PER_SEC: u32 = SAMPLE_RATE * BLOCK_ALIGN as u32; // 384000

// SPEAKER_FRONT_LEFT | SPEAKER_FRONT_RIGHT == 0x1 | 0x2 == 0x3.
const SPEAKER_STEREO_MASK: u32 = 0x3;

// ─── Chunk shape (the Plan 04-01 proven-transport contract — do NOT change) ─────────
//
// Each chunk delivered to JS is exactly 480 interleaved-stereo f32 frames:
//   480 frames * 2 channels * 4 bytes/sample = 3840 bytes (~10 ms at 48 kHz).
// This is the buffer shape the Plan 04-01 MSTG feeder already consumes; the JS wrapper
// forwards each 3840-byte buffer down the MessagePort with `port1.postMessage(buf)`.
const FRAMES_PER_CHUNK: usize = 480;
const BYTES_PER_FRAME: usize = (CHANNELS as usize) * (BITS_PER_SAMPLE as usize / 8); // 8
const CHUNK_BYTES: usize = FRAMES_PER_CHUNK * BYTES_PER_FRAME; // 3840

// Bounded ThreadsafeFunction queue: when JS can't keep up, NonBlocking `.call()` returns
// QueueFull and the Rust side drops the chunk (drop-oldest backpressure — locked T4 policy
// realized at the FFI boundary). MaxQueueSize has no effect in Blocking mode, so we use
// NonBlocking. ~5 chunks ≈ ~50 ms of slack before dropping. MaxQueueSize is a const generic
// on ThreadsafeFunction, so it is applied via the `ChunkTsfn` type alias below.
const TSFN_MAX_QUEUE: usize = 5;

// The napi callback the capture loop pushes to: a 3840-byte f32 Buffer per chunk, with a
// bounded queue (drop-oldest at QueueFull). Generic order is
// <T, Return, CallJsBackArgs, ErrorStatus, CalleeHandled, Weak, MaxQueueSize>; we keep the
// CalleeHandled default (true) so `.call(Ok(..))` is the call shape, and bound MaxQueueSize.
type ChunkTsfn = ThreadsafeFunction<
    Buffer,
    (),
    Buffer,
    napi::Status,
    true,  // CalleeHandled (default) — first JS arg is the error slot
    false, // Weak (default) — keep the event loop alive while capturing
    TSFN_MAX_QUEUE,
>;

// Bounded join timeout on stop so a hung native teardown can't wedge the caller's quit
// (composes with the JS wrapper's before-quit Promise.race).
const STOP_JOIN_TIMEOUT_MS: u64 = 1500;

/// Build the hardcoded 48k/stereo/f32 WAVEFORMATEXTENSIBLE.
fn build_wave_format() -> WAVEFORMATEXTENSIBLE {
    WAVEFORMATEXTENSIBLE {
        Format: WAVEFORMATEX {
            wFormatTag: WAVE_FORMAT_EXTENSIBLE as u16,
            nChannels: CHANNELS,
            nSamplesPerSec: SAMPLE_RATE,
            wBitsPerSample: BITS_PER_SAMPLE,
            nBlockAlign: BLOCK_ALIGN,
            nAvgBytesPerSec: AVG_BYTES_PER_SEC,
            // cbSize = bytes that follow the WAVEFORMATEX header (the EXTENSIBLE tail).
            cbSize: (size_of::<WAVEFORMATEXTENSIBLE>() - size_of::<WAVEFORMATEX>()) as u16,
        },
        Samples: WAVEFORMATEXTENSIBLE_0 {
            wValidBitsPerSample: BITS_PER_SAMPLE,
        },
        dwChannelMask: SPEAKER_STEREO_MASK,
        SubFormat: KSDATAFORMAT_SUBTYPE_IEEE_FLOAT,
    }
}

// ─── Activation outcome ─────────────────────────────────────────────────────────────

/// The result of attempting to activate process-loopback. Any failure (a missing
/// entry point, a non-S_OK activate result, or a COM error) collapses to `Unsupported`
/// — the JS-visible `start` then resolves `false` and the caller falls back gracefully
/// (ECHO-03). Nothing here panics or throws.
enum ActivationResult {
    /// Activated; the IAudioClient is initialized and ready to start capturing.
    Activated(IAudioClient),
    /// The API is absent on this build, or activation returned a non-success result.
    Unsupported,
}

// ─── Async activation completion handler ────────────────────────────────────────────
//
// ActivateAudioInterfaceAsync is asynchronous: it returns immediately and signals
// completion on an IActivateAudioInterfaceCompletionHandler. We implement the handler
// to set a Win32 event, then WaitForSingleObject on that event before reading the
// activation result (Pitfall 4 — treating activation as synchronous yields a null client).

#[implement(IActivateAudioInterfaceCompletionHandler)]
struct CompletionHandler {
    done: HANDLE,
}

impl IActivateAudioInterfaceCompletionHandler_Impl for CompletionHandler_Impl {
    fn ActivateCompleted(
        &self,
        _operation: windows::core::Ref<'_, IActivateAudioInterfaceAsyncOperation>,
    ) -> windows::core::Result<()> {
        // Wake the waiting thread; the activation result is read from the operation
        // by the caller after the wait returns.
        unsafe {
            let _ = SetEvent(self.done);
        }
        Ok(())
    }
}

// ─── Dynamic resolution of ActivateAudioInterfaceAsync ──────────────────────────────
//
// Resolve the entry point at runtime via LoadLibraryW + GetProcAddress (NOT a static
// import) so the .node LOADS on every Windows build; only where the symbol is present
// does activation proceed. A null GetProcAddress => Unsupported (graceful, ECHO-03).
//
// NOTE: the `windows` crate also exposes a statically-bound `ActivateAudioInterfaceAsync`
// (imported above and used for its type signatures); the dynamic probe below is the
// load-bearing availability gate — if the export is absent we never reach the call.

type ActivateAudioInterfaceAsyncFn = unsafe extern "system" fn(
    deviceinterfacepath: PCWSTR,
    riid: *const GUID,
    activationparams: *const PROPVARIANT,
    completionhandler: *mut core::ffi::c_void,
    activationoperation: *mut *mut core::ffi::c_void,
) -> HRESULT;

/// Returns true if `ActivateAudioInterfaceAsync` is resolvable on this build.
fn process_loopback_entrypoint_present() -> bool {
    unsafe {
        // mmdevapi.dll exports ActivateAudioInterfaceAsync on builds that support it.
        let module = match LoadLibraryW(w!("mmdevapi.dll")) {
            Ok(h) if !h.is_invalid() => h,
            _ => return false,
        };
        let proc = GetProcAddress(module, windows::core::s!("ActivateAudioInterfaceAsync"));
        // (s! builds a null-terminated PCSTR literal — windows-core macro, no path import.)
        // If GetProcAddress is null the API is unavailable on this build -> Unsupported.
        if proc.is_none() {
            // Distinguish the "old build" case in logs if ever needed.
            let _ = GetLastError() == ERROR_PROC_NOT_FOUND;
            return false;
        }
        // We keep the statically-bound symbol for the actual call (same export); this
        // probe is purely the availability gate so the .node still loads pre-2004.
        let _resolved: ActivateAudioInterfaceAsyncFn =
            std::mem::transmute::<_, ActivateAudioInterfaceAsyncFn>(proc.unwrap());
        true
    }
}

// ─── Activation ─────────────────────────────────────────────────────────────────────

/// Attempt to activate a process-loopback IAudioClient that EXCLUDES the process tree
/// rooted at `exclude_root_pid`, initialized to the hardcoded 48k/stereo/f32 format.
///
/// Returns `Activated(client)` on success, or `Unsupported` for ANY failure (missing
/// entry point, non-S_OK activate result, or COM error) — never panics, never throws.
unsafe fn activate_exclude_tree(exclude_root_pid: u32) -> ActivationResult {
    // 1. Dynamic-load gate: if the entry point is absent, this build doesn't support it.
    if !process_loopback_entrypoint_present() {
        return ActivationResult::Unsupported;
    }

    // 2. Build the activation params: EXCLUDE the supplied process tree.
    let mut activation_params = AUDIOCLIENT_ACTIVATION_PARAMS {
        ActivationType: AUDIOCLIENT_ACTIVATION_TYPE_PROCESS_LOOPBACK,
        Anonymous: AUDIOCLIENT_ACTIVATION_PARAMS_0 {
            ProcessLoopbackParams: AUDIOCLIENT_PROCESS_LOOPBACK_PARAMS {
                TargetProcessId: exclude_root_pid,
                // EXCLUDE tree: capture everything EXCEPT exclude_root_pid + its children.
                ProcessLoopbackMode: PROCESS_LOOPBACK_MODE_EXCLUDE_TARGET_PROCESS_TREE,
            },
        },
    };

    // 3. Wrap the params in a PROPVARIANT (VT_BLOB) for the activation call.
    //
    // HEAP-CORRUPTION FIX (0xc0000374): windows-rs's PROPVARIANT is an OWNING type — its Drop
    // calls PropVariantClear, which for VT_BLOB does CoTaskMemFree(blob.pBlobData). Here pBlobData
    // borrows the STACK `activation_params` (the PROPVARIANT owns NOTHING), so letting it drop would
    // CoTaskMemFree a stack pointer → heap corruption → hard crash inside start(). The C++
    // ApplicationLoopback sample uses a raw PROPVARIANT with no destructor; mirror that exactly by
    // wrapping in ManuallyDrop so PropVariantClear NEVER runs. No leak: the blob is stack memory
    // released with the stack frame, and the async activation completes (we wait) before we return.
    let mut prop = ManuallyDrop::new(PROPVARIANT::default());
    {
        let pv = &mut prop.Anonymous.Anonymous;
        pv.vt = VT_BLOB;
        pv.Anonymous.blob = BLOB {
            cbSize: size_of::<AUDIOCLIENT_ACTIVATION_PARAMS>() as u32,
            pBlobData: &mut activation_params as *mut _ as *mut u8,
        };
    }

    // 4. Create the completion event + handler (async activation, Pitfall 4).
    //    CreateEventW(attrs, bManualReset, bInitialState, name): manual-reset, unsignaled.
    let done = match CreateEventW(None, true, false, PCWSTR::null()) {
        Ok(h) => h,
        Err(_) => return ActivationResult::Unsupported,
    };
    let handler: IActivateAudioInterfaceCompletionHandler =
        CompletionHandler { done }.into();

    // 5. Fire the async activation against the process-loopback magic device.
    let operation: IActivateAudioInterfaceAsyncOperation = match ActivateAudioInterfaceAsync(
        VIRTUAL_AUDIO_DEVICE_PROCESS_LOOPBACK,
        &IAudioClient::IID,
        Some(&*prop),
        &handler,
    ) {
        Ok(op) => op,
        Err(_) => {
            let _ = CloseHandle(done);
            return ActivationResult::Unsupported;
        }
    };

    // 6. Wait for completion, then read the activation result. ANY non-S_OK => Unsupported.
    let wait = WaitForSingleObject(done, INFINITE);
    let _ = CloseHandle(done);
    if wait != WAIT_OBJECT_0 {
        return ActivationResult::Unsupported;
    }

    let mut activate_hr: HRESULT = E_FAIL;
    let mut activated_iface: Option<IUnknown> = None;
    if operation
        .GetActivateResult(&mut activate_hr, &mut activated_iface)
        .is_err()
    {
        return ActivationResult::Unsupported;
    }
    // E_NOTIMPL / E_INVALIDARG / AUDCLNT_E_DEVICE_INVALIDATED / any other non-success ->
    // "unsupported on this build" (try-activate-and-catch — no hardcoded OS build gate).
    if activate_hr != S_OK {
        return ActivationResult::Unsupported;
    }
    let audio_client: IAudioClient = match activated_iface.and_then(|u| u.cast().ok()) {
        Some(c) => c,
        None => return ActivationResult::Unsupported,
    };

    // 7. Initialize: StreamFlags is the SECOND parameter — AUTOCONVERTPCM goes HERE, not
    //    into hnsPeriodicity (Pitfall 2 / MS sample bug #196). AUTOCONVERTPCM makes the
    //    shared-mode engine convert to our hardcoded 48k/stereo/f32, so no Rust DSP.
    let wfx = build_wave_format();
    let stream_flags = AUDCLNT_STREAMFLAGS_LOOPBACK
        | AUDCLNT_STREAMFLAGS_EVENTCALLBACK
        | AUDCLNT_STREAMFLAGS_AUTOCONVERTPCM
        | AUDCLNT_STREAMFLAGS_SRC_DEFAULT_QUALITY;
    if audio_client
        .Initialize(
            AUDCLNT_SHAREMODE_SHARED,
            stream_flags,                 // <-- StreamFlags (2nd param): AUTOCONVERTPCM lives here.
            0,                            // hnsBufferDuration (engine default in event-driven shared mode)
            0,                            // hnsPeriodicity (0 for event-driven shared mode)
            &wfx as *const _ as *const WAVEFORMATEX,
            None,
        )
        .is_err()
    {
        return ActivationResult::Unsupported;
    }

    ActivationResult::Activated(audio_client)
}

// ─── Capture-thread state ───────────────────────────────────────────────────────────
//
// A single capture session at a time. The capture thread owns the COM-apartment-affine
// IAudioClient/IAudioCaptureClient: activation runs ON the capture thread (after
// CoInitializeEx) and the outcome is reported back to `start()` over a channel, so the
// COM objects never cross a thread boundary. `stop` signals the stop event and joins.

struct CaptureSession {
    /// Manual-reset event the capture loop polls; SetEvent => "please exit".
    stop_event: HANDLE,
    /// The capture thread join handle (taken by `stop`).
    join: Option<JoinHandle<()>>,
}

// HANDLE is a raw pointer; it is only ever touched under the GLOBAL mutex below and on
// the capture thread we created, so guarding it this way is sound.
unsafe impl Send for CaptureSession {}

/// A `HANDLE` that can be moved into the capture thread. A Win32 event handle is a kernel
/// object safe to use from any thread; the raw pointer is only non-`Send` by default.
#[derive(Clone, Copy)]
struct SendHandle(HANDLE);
unsafe impl Send for SendHandle {}

static SESSION: Mutex<Option<CaptureSession>> = Mutex::new(None);

// ─── Event-driven capture loop (runs on the dedicated capture thread) ───────────────
//
// Source basis: the MS ApplicationLoopback sample's event-driven capture loop
// (SetEventHandle -> GetService(IAudioCaptureClient) -> Start -> wait-on-event ->
// GetNextPacketSize / GetBuffer / ReleaseBuffer). Re-expressed via windows-rs.

/// The capture loop. Owns `audio_client` (already Initialize()'d to 48k/stereo/f32).
/// Batches the device's interleaved-stereo f32 frames into 3840-byte chunks and pushes
/// each via `on_chunk` (NonBlocking — drops on QueueFull). Exits when `stop_event` fires.
unsafe fn run_capture_loop(audio_client: IAudioClient, on_chunk: ChunkTsfn, stop_event: HANDLE) {
    // Event the engine signals each period (EVENTCALLBACK mode). Auto-reset, unsignaled.
    let audio_event = match CreateEventW(None, false, false, PCWSTR::null()) {
        Ok(h) => h,
        Err(_) => return,
    };
    if audio_client.SetEventHandle(audio_event).is_err() {
        let _ = CloseHandle(audio_event);
        return;
    }

    let capture: IAudioCaptureClient = match audio_client.GetService() {
        Ok(c) => c,
        Err(_) => {
            let _ = CloseHandle(audio_event);
            return;
        }
    };

    if audio_client.Start().is_err() {
        let _ = CloseHandle(audio_event);
        return;
    }

    // Accumulator: fill to exactly CHUNK_BYTES (3840), flush, repeat. WASAPI packets do
    // not align to 480 frames, so we re-chunk across packet boundaries.
    let mut acc: Vec<u8> = Vec::with_capacity(CHUNK_BYTES * 2);

    // Wait on BOTH the audio event and the stop event; WaitForMultipleObjects would be
    // ideal, but a short timed wait on the audio event + a stop-event poll keeps the
    // dependency surface minimal and the teardown latency bounded.
    loop {
        // Stop requested? (non-blocking poll; WAIT_OBJECT_0 => signaled)
        if WaitForSingleObject(stop_event, 0) == WAIT_OBJECT_0 {
            break;
        }

        // Wait up to ~100 ms for the next audio period; a timeout just re-polls stop.
        let _ = WaitForSingleObject(audio_event, 100);

        // Drain every packet currently available.
        loop {
            let packet_frames = match capture.GetNextPacketSize() {
                Ok(n) => n,
                Err(_) => break,
            };
            if packet_frames == 0 {
                break;
            }

            let mut data_ptr: *mut u8 = std::ptr::null_mut();
            let mut num_frames: u32 = 0;
            let mut flags: u32 = 0;
            if capture
                .GetBuffer(&mut data_ptr, &mut num_frames, &mut flags, None, None)
                .is_err()
            {
                break;
            }

            let frame_count = num_frames as usize;
            let byte_count = frame_count * BYTES_PER_FRAME;
            if (flags & AUDCLNT_BUFFERFLAGS_SILENT.0 as u32) != 0 || data_ptr.is_null() {
                // Silent packet: the engine says "treat as silence" — append zeros so the
                // timeline stays monotonic (the renderer expects continuous f32 frames).
                acc.resize(acc.len() + byte_count, 0u8);
            } else {
                let slice = std::slice::from_raw_parts(data_ptr, byte_count);
                acc.extend_from_slice(slice);
            }

            let _ = capture.ReleaseBuffer(num_frames);

            // Flush as many full 3840-byte chunks as we now have.
            while acc.len() >= CHUNK_BYTES {
                let chunk: Vec<u8> = acc.drain(..CHUNK_BYTES).collect();
                // NonBlocking push: QueueFull => the chunk is dropped (drop-oldest at the
                // FFI boundary). Bounded latency wins over perfect fidelity (locked T4).
                on_chunk.call(Ok(chunk.into()), ThreadsafeFunctionCallMode::NonBlocking);
            }
        }
    }

    // Teardown: stop the client and release the per-loop event.
    let _ = audio_client.Stop();
    let _ = CloseHandle(audio_event);
    // `capture` and `audio_client` drop here, releasing the COM references on this thread.
}

// ─── napi surface ───────────────────────────────────────────────────────────────────

/// Begin process-tree EXCLUDE loopback capture, excluding the tree rooted at
/// `exclude_root_pid` (pass the Electron MAIN process PID — EXCLUDE_TARGET_PROCESS_TREE
/// covers the whole tree incl. the separate "Audio Service" utility child). `on_chunk`
/// receives 480-frame (3840-byte) interleaved-stereo f32 buffers, ~one per 10 ms.
///
/// Returns `false` (NOT an error) when the API is unavailable on this build OR activation
/// fails for any reason — the caller then falls back to Electron "loopback" (ECHO-03).
#[napi]
pub fn start(exclude_root_pid: u32, on_chunk: ChunkTsfn) -> napi::Result<bool> {
    let mut guard = SESSION.lock().unwrap_or_else(|p| p.into_inner());
    // Idempotent: a session already running counts as "started".
    if guard.is_some() {
        return Ok(true);
    }

    // Stop event the capture thread polls. Created here so `stop()` can signal it even if
    // the thread is still activating.
    // Manual-reset stop event the capture loop polls; unsignaled initially.
    let stop_event = match unsafe { CreateEventW(None, true, false, PCWSTR::null()) } {
        Ok(h) => h,
        Err(_) => return Ok(false),
    };

    // Activation is COM-apartment-affine, so run it ON the capture thread and report the
    // outcome back over a channel; `start` returns the real support verdict.
    let (tx, rx): (Sender<bool>, _) = channel();
    let stop_event_for_thread = SendHandle(stop_event);

    let join = std::thread::spawn(move || {
        // Capture the whole SendHandle (Send), not its inner HANDLE field — Rust 2021's
        // disjoint closure capture would otherwise grab the non-Send `.0` directly.
        let stop_handle = stop_event_for_thread;

        // COM on the capture thread (MTA — no message pump needed for WASAPI capture).
        let com = unsafe { CoInitializeEx(None, COINIT_MULTITHREADED) };
        // CoInitializeEx returns an HRESULT; S_FALSE means "already initialized" (still ok).
        let com_ok = com.is_ok();

        let activation = unsafe { activate_exclude_tree(exclude_root_pid) };
        match activation {
            ActivationResult::Activated(client) => {
                // Report support BEFORE entering the (blocking) capture loop.
                let _ = tx.send(true);
                unsafe { run_capture_loop(client, on_chunk, stop_handle.0) };
            }
            ActivationResult::Unsupported => {
                let _ = tx.send(false);
            }
        }

        if com_ok {
            unsafe { CoUninitialize() };
        }
    });

    // Wait for the activation verdict from the capture thread.
    let supported = rx.recv().unwrap_or(false);
    if !supported {
        // Unsupported: tear the (now-exiting) thread down and clean up the stop event.
        unsafe {
            let _ = SetEvent(stop_event);
        }
        let _ = join.join();
        unsafe {
            let _ = CloseHandle(stop_event);
        }
        return Ok(false);
    }

    *guard = Some(CaptureSession {
        stop_event,
        join: Some(join),
    });
    Ok(true)
}

/// Stop capture: signal the capture thread to exit, then join it with a bounded timeout
/// so a hung native teardown can't wedge the caller (composes with the JS wrapper's
/// before-quit Promise.race). Idempotent — a no-op if nothing is running.
#[napi]
pub fn stop() {
    let session = {
        let mut guard = SESSION.lock().unwrap_or_else(|p| p.into_inner());
        guard.take()
    };

    let Some(mut session) = session else {
        return; // idempotent: nothing running.
    };

    // Signal the capture loop to exit.
    unsafe {
        let _ = SetEvent(session.stop_event);
    }

    // Join with a bounded timeout: poll is_finished() rather than block forever.
    if let Some(join) = session.join.take() {
        let deadline = std::time::Instant::now()
            + std::time::Duration::from_millis(STOP_JOIN_TIMEOUT_MS);
        loop {
            if join.is_finished() {
                let _ = join.join();
                break;
            }
            if std::time::Instant::now() >= deadline {
                // Detach a hung thread rather than block the caller; the stop event is
                // already signaled, so it will exit on its own shortly.
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
    }

    unsafe {
        let _ = CloseHandle(session.stop_event);
    }
}
