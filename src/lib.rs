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
//
// Startup contract: a start* call returns a session id only once the stream is RUNNING
// (activation, Initialize, SetEventHandle, GetService and Start all succeeded), and it never
// blocks longer than STARTUP_TIMEOUT_MS. A stream that dies later (device invalidated, …) ends
// its thread and reports one error through the chunk callback instead of spinning forever.

#![cfg(windows)]

use std::collections::HashMap;
use std::mem::{size_of, ManuallyDrop};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::mpsc::{channel, Sender};
use std::sync::{Arc, LazyLock, Mutex, OnceLock};
use std::thread::JoinHandle;
use std::time::Duration;

use napi::bindgen_prelude::Buffer;
use napi::threadsafe_function::{ThreadsafeFunction, ThreadsafeFunctionCallMode};
use napi_derive::napi;

// The `#[implement]` macro emits absolute `::windows_core::` paths, so windows-core is a
// direct dependency (see Cargo.toml). `windows` also re-exports it as `windows::core`.
use windows::core::{implement, w, IUnknown, Interface, GUID, HRESULT, PCWSTR, PWSTR};
use windows::Win32::Foundation::{CloseHandle, E_FAIL, HANDLE, S_OK, WAIT_OBJECT_0, WAIT_TIMEOUT};
use windows::Win32::Media::Audio::{
    IActivateAudioInterfaceAsyncOperation,
    IActivateAudioInterfaceCompletionHandler, IActivateAudioInterfaceCompletionHandler_Impl,
    IAudioCaptureClient, IAudioClient, IAudioSessionControl, IAudioSessionControl2,
    IAudioSessionEnumerator, IAudioSessionManager2, IMMDevice, IMMDeviceCollection,
    IMMDeviceEnumerator, AUDCLNT_BUFFERFLAGS_DATA_DISCONTINUITY, AUDCLNT_BUFFERFLAGS_SILENT,
    AUDCLNT_BUFFERFLAGS_TIMESTAMP_ERROR, AUDCLNT_SHAREMODE_SHARED,
    AUDCLNT_STREAMFLAGS_AUTOCONVERTPCM, AUDCLNT_STREAMFLAGS_EVENTCALLBACK,
    AUDCLNT_STREAMFLAGS_LOOPBACK, AUDCLNT_STREAMFLAGS_SRC_DEFAULT_QUALITY,
    AUDIOCLIENT_ACTIVATION_PARAMS, AUDIOCLIENT_ACTIVATION_PARAMS_0,
    AUDIOCLIENT_ACTIVATION_TYPE_PROCESS_LOOPBACK, AUDIOCLIENT_PROCESS_LOOPBACK_PARAMS,
    DEVICE_STATE_ACTIVE, MMDeviceEnumerator, PROCESS_LOOPBACK_MODE,
    PROCESS_LOOPBACK_MODE_EXCLUDE_TARGET_PROCESS_TREE,
    PROCESS_LOOPBACK_MODE_INCLUDE_TARGET_PROCESS_TREE, VIRTUAL_AUDIO_DEVICE_PROCESS_LOOPBACK,
    WAVEFORMATEX, WAVEFORMATEXTENSIBLE, WAVEFORMATEXTENSIBLE_0, eConsole, eRender,
};
use windows::Win32::Media::KernelStreaming::WAVE_FORMAT_EXTENSIBLE;
use windows::Win32::Media::Multimedia::KSDATAFORMAT_SUBTYPE_IEEE_FLOAT;
// Endpoint friendly-name property key + the property store interface returned by
// IMMDevice::OpenPropertyStore (render-endpoint enumeration). PKEY_Device_FriendlyName lives under
// Win32_Devices_FunctionDiscovery; IPropertyStore under Win32_UI_Shell_PropertiesSystem.
use windows::Win32::Devices::FunctionDiscovery::PKEY_Device_FriendlyName;
use windows::Win32::UI::Shell::PropertiesSystem::IPropertyStore;
use windows::Win32::System::Com::StructuredStorage::PROPVARIANT;
use windows::Win32::System::Com::{
    CoCreateInstance, CoInitializeEx, CoTaskMemFree, CoUninitialize, BLOB, CLSCTX_ALL,
    COINIT_MULTITHREADED, STGM_READ,
};
use windows::Win32::System::LibraryLoader::{GetProcAddress, LoadLibraryW};
use windows::Win32::System::Threading::{
    CreateEventW, GetCurrentProcessId, OpenProcess, QueryFullProcessImageNameW, SetEvent,
    WaitForSingleObject, PROCESS_NAME_WIN32, PROCESS_QUERY_LIMITED_INFORMATION,
};
use windows::Win32::System::Variant::{VT_BLOB, VT_LPWSTR};

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

// Bounded wait for the async activation callback. Activation normally completes in well under
// 100 ms; a wedged audio service must not hang the JS thread that called start*.
const ACTIVATION_TIMEOUT_MS: u32 = 3000;

// Bounded wait in `spawn_session` for the capture thread's verdict (activation + Initialize +
// stream start). Longer than the activation timeout so a slow-but-finishing activation still wins.
const STARTUP_TIMEOUT_MS: u64 = ACTIVATION_TIMEOUT_MS as u64 + 2000;

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
//
// LIFETIME: the wait is bounded, so the caller may give up and return while activation is still
// in flight. Everything the activation can still touch — the completion event, the activation
// params and the PROPVARIANT pointing at them — is therefore owned by the handler, not the
// caller's stack. The pending operation holds a COM reference to the handler until it completes,
// so these are freed (Drop below) only once nothing can use them any more.

#[implement(IActivateAudioInterfaceCompletionHandler)]
struct CompletionHandler {
    done: HANDLE,
    params: *mut AUDIOCLIENT_ACTIVATION_PARAMS,
    prop: *mut ManuallyDrop<PROPVARIANT>,
}

impl Drop for CompletionHandler {
    fn drop(&mut self) {
        unsafe {
            let _ = CloseHandle(self.done);
            // Freed through ManuallyDrop: PropVariantClear must NEVER run on this PROPVARIANT,
            // because for VT_BLOB it would CoTaskMemFree `params`, which Rust allocated
            // (the 0xc0000374 heap-corruption crash).
            drop(Box::from_raw(self.prop));
            drop(Box::from_raw(self.params));
        }
    }
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
// import) and call it through the resolved pointer, so the .node carries no import of the
// symbol and LOADS on every Windows build; only where the symbol is present does activation
// proceed. A null GetProcAddress => Unsupported (graceful, ECHO-03).

type ActivateAudioInterfaceAsyncFn = unsafe extern "system" fn(
    deviceinterfacepath: PCWSTR,
    riid: *const GUID,
    activationparams: *const PROPVARIANT,
    completionhandler: *mut core::ffi::c_void,
    activationoperation: *mut *mut core::ffi::c_void,
) -> HRESULT;

/// `ActivateAudioInterfaceAsync` from mmdevapi.dll, or `None` where this build lacks it.
/// Resolved once; the module is deliberately never freed so the pointer stays valid.
fn resolve_activate_fn() -> Option<ActivateAudioInterfaceAsyncFn> {
    static RESOLVED: OnceLock<Option<ActivateAudioInterfaceAsyncFn>> = OnceLock::new();
    *RESOLVED.get_or_init(|| unsafe {
        let module = LoadLibraryW(w!("mmdevapi.dll")).ok().filter(|h| !h.is_invalid())?;
        // (s! builds a null-terminated PCSTR literal — windows-core macro, no path import.)
        let proc = GetProcAddress(module, windows::core::s!("ActivateAudioInterfaceAsync"))?;
        Some(std::mem::transmute::<unsafe extern "system" fn() -> isize, ActivateAudioInterfaceAsyncFn>(proc))
    })
}

// ─── Activation ─────────────────────────────────────────────────────────────────────

/// Initialize an already-acquired `IAudioClient` to the fixed 48k/stereo/f32 loopback format.
///
/// This is the SINGLE Initialize/stream-flags block shared by BOTH client-acquisition paths —
/// process-tree activation (the magic device) and render-endpoint activation (a real `IMMDevice`).
/// Routing both through here guarantees the transport format never diverges: shared mode with
/// `AUTOCONVERTPCM | SRC_DEFAULT_QUALITY` so the engine converts to our hardcoded format (the
/// deliberate divergence from OBS, which queries the endpoint mix format), plus
/// `LOOPBACK | EVENTCALLBACK` for the event-driven capture loop. StreamFlags is the SECOND
/// Initialize parameter — AUTOCONVERTPCM goes HERE, not into hnsPeriodicity (Pitfall 2 / MS
/// sample bug #196). Returns the raw `Initialize` result; callers collapse `Err` to `Unsupported`.
unsafe fn initialize_loopback_client(audio_client: &IAudioClient) -> windows::core::Result<()> {
    let wfx = build_wave_format();
    let stream_flags = AUDCLNT_STREAMFLAGS_LOOPBACK
        | AUDCLNT_STREAMFLAGS_EVENTCALLBACK
        | AUDCLNT_STREAMFLAGS_AUTOCONVERTPCM
        | AUDCLNT_STREAMFLAGS_SRC_DEFAULT_QUALITY;
    audio_client.Initialize(
        AUDCLNT_SHAREMODE_SHARED,
        stream_flags,                 // <-- StreamFlags (2nd param): AUTOCONVERTPCM lives here.
        0,                            // hnsBufferDuration (engine default in event-driven shared mode)
        0,                            // hnsPeriodicity (0 for event-driven shared mode)
        &wfx as *const _ as *const WAVEFORMATEX,
        None,
    )
}

/// Attempt to activate a process-loopback IAudioClient for the process tree rooted at
/// `target_pid`, in the requested `mode`, initialized to the hardcoded 48k/stereo/f32 format.
///
/// `mode` is the ONLY functional divergence between the two capture kinds:
///   - `EXCLUDE_TARGET_PROCESS_TREE` — capture everything EXCEPT `target_pid`'s tree (the
///     shipped #46 echo fix: pass the Electron main PID to drop the host's own playback).
///   - `INCLUDE_TARGET_PROCESS_TREE` — capture ONLY `target_pid`'s tree (app-exclusive share).
/// Every other step (VT_BLOB guard, async wait, `build_wave_format()` + `AUTOCONVERTPCM`
/// initialize) is byte-identical across both modes.
///
/// Returns `Activated(client)` on success, or `Unsupported` for ANY failure (missing
/// entry point, non-S_OK activate result, or COM error) — never panics, never throws.
unsafe fn activate_process_tree(mode: PROCESS_LOOPBACK_MODE, target_pid: u32) -> ActivationResult {
    // 1. Dynamic-load gate: if the entry point is absent, this build doesn't support it.
    let Some(activate) = resolve_activate_fn() else {
        return ActivationResult::Unsupported;
    };

    // Completion event: manual-reset, unsignaled. Owned by the handler from step 4 on.
    let done = match CreateEventW(None, true, false, PCWSTR::null()) {
        Ok(h) => h,
        Err(_) => return ActivationResult::Unsupported,
    };

    // 2. Build the activation params for the supplied process tree in the requested mode.
    let params = Box::into_raw(Box::new(AUDIOCLIENT_ACTIVATION_PARAMS {
        ActivationType: AUDIOCLIENT_ACTIVATION_TYPE_PROCESS_LOOPBACK,
        Anonymous: AUDIOCLIENT_ACTIVATION_PARAMS_0 {
            ProcessLoopbackParams: AUDIOCLIENT_PROCESS_LOOPBACK_PARAMS {
                TargetProcessId: target_pid,
                // EXCLUDE tree: capture everything EXCEPT target_pid + its children.
                // INCLUDE tree: capture ONLY target_pid + its children.
                ProcessLoopbackMode: mode,
            },
        },
    }));

    // 3. Wrap the params in a PROPVARIANT (VT_BLOB) for the activation call.
    //
    // HEAP-CORRUPTION FIX (0xc0000374): windows-rs's PROPVARIANT is an OWNING type — its Drop
    // calls PropVariantClear, which for VT_BLOB does CoTaskMemFree(blob.pBlobData). pBlobData
    // borrows `params` (the PROPVARIANT owns NOTHING), so letting it drop would CoTaskMemFree a
    // Rust allocation → heap corruption → hard crash inside start(). The C++ ApplicationLoopback
    // sample uses a raw PROPVARIANT with no destructor; mirror that with ManuallyDrop so
    // PropVariantClear NEVER runs.
    let mut prop = ManuallyDrop::new(PROPVARIANT::default());
    {
        let pv = &mut prop.Anonymous.Anonymous;
        pv.vt = VT_BLOB;
        pv.Anonymous.blob = BLOB {
            cbSize: size_of::<AUDIOCLIENT_ACTIVATION_PARAMS>() as u32,
            pBlobData: params as *mut u8,
        };
    }
    let prop = Box::into_raw(Box::new(prop));

    // 4. The handler takes ownership of the event, params and PROPVARIANT (see LIFETIME above).
    let handler: IActivateAudioInterfaceCompletionHandler =
        CompletionHandler { done, params, prop }.into();

    // 5. Fire the async activation against the process-loopback magic device, through the
    //    resolved pointer.
    let mut operation_raw: *mut core::ffi::c_void = std::ptr::null_mut();
    let hr = activate(
        VIRTUAL_AUDIO_DEVICE_PROCESS_LOOPBACK,
        &IAudioClient::IID,
        prop as *const PROPVARIANT,
        handler.as_raw(),
        &mut operation_raw,
    );
    // Take ownership of any returned operation so it is released on every path.
    let operation = (!operation_raw.is_null())
        .then(|| IActivateAudioInterfaceAsyncOperation::from_raw(operation_raw));
    let operation = match operation {
        Some(op) if hr.is_ok() => op,
        _ => return ActivationResult::Unsupported,
    };

    // 6. Wait (bounded) for completion, then read the activation result. ANY non-S_OK =>
    //    Unsupported. On timeout we just drop our references: the pending operation keeps the
    //    handler — and with it the event and params — alive until it completes.
    if WaitForSingleObject(done, ACTIVATION_TIMEOUT_MS) != WAIT_OBJECT_0 {
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

    // 7. Initialize to the fixed 48k/stereo/f32 loopback format via the shared helper — the SAME
    //    Initialize/stream-flags block the endpoint path uses, so the transport format stays
    //    byte-identical across both client-acquisition paths. AUTOCONVERTPCM makes the shared-mode
    //    engine convert to our hardcoded format, so no Rust DSP.
    if initialize_loopback_client(&audio_client).is_err() {
        return ActivationResult::Unsupported;
    }

    ActivationResult::Activated(audio_client)
}

// ─── Capture-thread state ───────────────────────────────────────────────────────────
//
// N concurrent capture sessions, keyed in SESSIONS. Each capture thread owns its own
// COM-apartment-affine
// IAudioClient/IAudioCaptureClient: activation runs ON the capture thread (after
// CoInitializeEx) and the outcome is reported back to `start()` over a channel, so the
// COM objects never cross a thread boundary. `stop` signals the stop event and joins.

struct CaptureSession {
    /// Manual-reset event the capture loop polls; SetEvent => "please exit".
    stop_event: HANDLE,
    /// The capture thread join handle (taken by `stop`).
    join: Option<JoinHandle<()>>,
    /// Live timing counters, shared with the capture thread.
    stats: Arc<StreamStats>,
}

// HANDLE is a raw pointer; it is only ever touched under the GLOBAL mutex below and on
// the capture thread we created, so guarding it this way is sound.
unsafe impl Send for CaptureSession {}

/// A `HANDLE` that can be moved into the capture thread. A Win32 event handle is a kernel
/// object safe to use from any thread; the raw pointer is only non-`Send` by default.
#[derive(Clone, Copy)]
struct SendHandle(HANDLE);
unsafe impl Send for SendHandle {}

const FLAG_SILENT: u32 = AUDCLNT_BUFFERFLAGS_SILENT.0 as u32;
const FLAG_DATA_DISCONTINUITY: u32 = AUDCLNT_BUFFERFLAGS_DATA_DISCONTINUITY.0 as u32;
const FLAG_TIMESTAMP_ERROR: u32 = AUDCLNT_BUFFERFLAGS_TIMESTAMP_ERROR.0 as u32;

/// Per-stream timing taken straight from `IAudioCaptureClient::GetBuffer`, published as ONE
/// coherent snapshot (under a lock) so a reader never pairs fields from different packets.
/// Diagnostics only — nothing here drives capture or subtraction.
///
/// `captured_frames` counts every frame the engine handed us (silent packets included), so it is
/// our own monotonic stream clock and it survives the FFI queue dropping chunks — which is why
/// inferring a rate from chunk *arrival* cannot work (`dropped_chunks` counts that directly).
/// The timing pair is (`timing_frame`, `timing_qpc_100ns`): the captured-frame index of the
/// first frame of the last packet with a valid timestamp, and the performance counter at which
/// the engine recorded it. `DevicePosition` is kept raw but is NOT trusted: process loopback has
/// been seen reporting 0 throughout, so `device_position_advances` says whether it ever moved.
#[derive(Clone, Copy, Default, Debug, PartialEq)]
struct TimingSnapshot {
    /// Bumped on every update; an unchanged value across polls means nothing new happened.
    sequence: u64,
    running: bool,
    /// HRESULT that ended the stream, 0 while running or after a requested stop.
    error_hresult: i32,
    packets: u64,
    captured_frames: u64,
    last_packet_frames: u32,
    silent_packets: u64,
    discontinuities: u64,
    timestamp_errors: u64,
    delivered_chunks: u64,
    dropped_chunks: u64,
    timing_valid: bool,
    timing_frame: u64,
    timing_qpc_100ns: u64,
    timing_device_position: u64,
    device_position_advances: u64,
}

impl TimingSnapshot {
    fn record_packet(&mut self, frames: u32, flags: u32, device_position: u64, qpc_position: u64) {
        self.sequence += 1;
        self.packets += 1;
        let first_frame = self.captured_frames;
        self.captured_frames += u64::from(frames);
        self.last_packet_frames = frames;
        if flags & FLAG_SILENT != 0 {
            self.silent_packets += 1;
        }
        if flags & FLAG_DATA_DISCONTINUITY != 0 {
            self.discontinuities += 1;
        }
        if flags & FLAG_TIMESTAMP_ERROR != 0 {
            self.timestamp_errors += 1;
            return; // keep the previous valid pair
        }
        if qpc_position == 0 {
            return;
        }
        if self.timing_valid && device_position > self.timing_device_position {
            self.device_position_advances += 1;
        }
        self.timing_valid = true;
        self.timing_frame = first_frame;
        self.timing_qpc_100ns = qpc_position;
        self.timing_device_position = device_position;
    }

    fn record_chunk(&mut self, delivered: bool) {
        self.sequence += 1;
        if delivered {
            self.delivered_chunks += 1;
        } else {
            self.dropped_chunks += 1;
        }
    }

    fn finish(&mut self, failure: Option<HRESULT>) {
        self.sequence += 1;
        self.running = false;
        self.error_hresult = failure.map_or(0, |hr| hr.0);
    }
}

#[derive(Default)]
struct StreamStats(Mutex<TimingSnapshot>);

impl StreamStats {
    fn update(&self, f: impl FnOnce(&mut TimingSnapshot)) {
        f(&mut self.0.lock().unwrap_or_else(|p| p.into_inner()));
    }

    fn snapshot(&self) -> TimingSnapshot {
        *self.0.lock().unwrap_or_else(|p| p.into_inner())
    }
}

/// All live capture sessions, keyed by the id handed back to JS.
///
/// This used to be a single `Mutex<Option<CaptureSession>>`. That singleton made multi-app
/// INCLUDE capture (Discord's window-share mode, generalized to N apps) impossible AND
/// silently wrong: a second `start*` call hit the `is_some()` early-return and reported
/// SUCCESS while capturing nothing. Keying by session id is what makes N concurrent
/// INCLUDE captures — one per selected app — actually run.
static SESSIONS: LazyLock<Mutex<HashMap<u32, CaptureSession>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

/// Monotonic session-id source. Starts at 1 so `0` is never a valid id and the TS side can
/// use `-1`/`0` as "failed" sentinels without ambiguity.
static NEXT_SESSION_ID: AtomicU32 = AtomicU32::new(1);

// ─── Event-driven capture loop (runs on the dedicated capture thread) ───────────────
//
// Source basis: the MS ApplicationLoopback sample's event-driven capture loop
// (SetEventHandle -> GetService(IAudioCaptureClient) -> Start -> wait-on-event ->
// GetNextPacketSize / GetBuffer / ReleaseBuffer). Re-expressed via windows-rs.

/// A started stream: the client is running and signalling `audio_event` each period.
struct RunningStream {
    audio_client: IAudioClient,
    capture: IAudioCaptureClient,
    audio_event: HANDLE,
}

/// SetEventHandle -> GetService -> Start on an Initialize()'d client. `None` if any step fails;
/// only a `Some` means the session may be reported as started.
unsafe fn start_stream(audio_client: IAudioClient) -> Option<RunningStream> {
    // Event the engine signals each period (EVENTCALLBACK mode). Auto-reset, unsignaled.
    let audio_event = CreateEventW(None, false, false, PCWSTR::null()).ok()?;
    if audio_client.SetEventHandle(audio_event).is_ok() {
        if let Ok(capture) = audio_client.GetService::<IAudioCaptureClient>() {
            if audio_client.Start().is_ok() {
                return Some(RunningStream {
                    audio_client,
                    capture,
                    audio_event,
                });
            }
        }
    }
    // Release the client (which holds the event) before closing the event.
    drop(audio_client);
    let _ = CloseHandle(audio_event);
    None
}

/// The capture loop over a started stream. Batches the device's interleaved-stereo f32 frames
/// into 3840-byte chunks and pushes each via `on_chunk` (NonBlocking — drops on QueueFull).
/// Exits when `stop_event` fires, or on the first capture-client error: a device error is
/// terminal (it repeats every period), so it ends the session and is reported once through
/// `on_chunk` as an error rather than looping forever producing nothing.
unsafe fn run_capture_loop(
    stream: RunningStream,
    on_chunk: ChunkTsfn,
    stop_event: HANDLE,
    stats: Arc<StreamStats>,
) {
    let RunningStream {
        audio_client,
        capture,
        audio_event,
    } = stream;

    // Accumulator: fill to exactly CHUNK_BYTES (3840), flush, repeat. WASAPI packets do
    // not align to 480 frames, so we re-chunk across packet boundaries.
    let mut acc: Vec<u8> = Vec::with_capacity(CHUNK_BYTES * 2);
    let mut failure: Option<HRESULT> = None;

    // Wait on BOTH the audio event and the stop event; WaitForMultipleObjects would be
    // ideal, but a short timed wait on the audio event + a stop-event poll keeps the
    // dependency surface minimal and the teardown latency bounded.
    'capture: loop {
        // Stop requested? (non-blocking poll.) Exit on ANYTHING that is not "still unsignaled".
        //
        // This deliberately treats WAIT_FAILED as "stop" too. If `stop()` hits its join timeout
        // it detaches this thread, and a closed/recycled stop handle then makes WaitForSingleObject
        // return WAIT_FAILED — the old `== WAIT_OBJECT_0` test would never match, so the detached
        // thread span forever, still pushing chunks into a dropped ThreadsafeFunction.
        if WaitForSingleObject(stop_event, 0) != WAIT_TIMEOUT {
            break;
        }

        // Wait up to ~100 ms for the next audio period; a timeout just re-polls stop.
        let woke = WaitForSingleObject(audio_event, 100);
        if woke != WAIT_OBJECT_0 && woke != WAIT_TIMEOUT {
            failure = Some(E_FAIL);
            break;
        }

        // Drain every packet currently available.
        loop {
            let packet_frames = match capture.GetNextPacketSize() {
                Ok(n) => n,
                Err(e) => {
                    failure = Some(e.code());
                    break 'capture;
                }
            };
            if packet_frames == 0 {
                break;
            }

            let mut data_ptr: *mut u8 = std::ptr::null_mut();
            let mut num_frames: u32 = 0;
            let mut flags: u32 = 0;
            // The two trailing arguments used to be None. They are the engine's timing for this
            // packet, and discarding them is what forced rate to be guessed from arrival times.
            let mut device_pos: u64 = 0;
            let mut qpc_pos: u64 = 0;
            if let Err(e) = capture.GetBuffer(
                &mut data_ptr,
                &mut num_frames,
                &mut flags,
                Some(&mut device_pos),
                Some(&mut qpc_pos),
            ) {
                failure = Some(e.code());
                break 'capture;
            }

            stats.update(|s| s.record_packet(num_frames, flags, device_pos, qpc_pos));

            let frame_count = num_frames as usize;
            let byte_count = frame_count * BYTES_PER_FRAME;
            if (flags & FLAG_SILENT) != 0 || data_ptr.is_null() {
                // Silent packet: the engine says "treat as silence" — append zeros so the
                // timeline stays monotonic (the renderer expects continuous f32 frames).
                acc.resize(acc.len() + byte_count, 0u8);
            } else {
                let slice = std::slice::from_raw_parts(data_ptr, byte_count);
                acc.extend_from_slice(slice);
            }

            if let Err(e) = capture.ReleaseBuffer(num_frames) {
                failure = Some(e.code());
                break 'capture;
            }

            // Flush as many full 3840-byte chunks as we now have.
            while acc.len() >= CHUNK_BYTES {
                let chunk: Vec<u8> = acc.drain(..CHUNK_BYTES).collect();
                // NonBlocking push: QueueFull => the chunk is dropped (drop-oldest at the
                // FFI boundary). Bounded latency wins over perfect fidelity (locked T4).
                let delivered = on_chunk
                    .call(Ok(chunk.into()), ThreadsafeFunctionCallMode::NonBlocking)
                    == napi::Status::Ok;
                stats.update(|s| s.record_chunk(delivered));
            }
        }
    }

    // Teardown: stop the client, release the COM references on this thread, and only then close
    // the event the client was signalling.
    let _ = audio_client.Stop();
    drop(capture);
    drop(audio_client);
    let _ = CloseHandle(audio_event);
    stats.update(|s| s.finish(failure));

    // Tell JS this session is gone for good (best-effort: NonBlocking may hit QueueFull, and a
    // Blocking call here could wedge against a caller joining this thread).
    if let Some(hr) = failure {
        let _ = on_chunk.call(
            Err(napi::Error::new(
                napi::Status::GenericFailure,
                format!("WASAPI capture stream ended: HRESULT 0x{:08X}", hr.0 as u32),
            )),
            ThreadsafeFunctionCallMode::NonBlocking,
        );
    }
}

// ─── Session spawner (shared by every capture mode) ─────────────────────────────────

/// Spawn one capture session: create its stop event, run `activate` ON a dedicated MTA
/// capture thread (COM apartment affinity — the IAudioClient never crosses a thread
/// boundary), wait for the activation verdict, then register the session in `SESSIONS`.
///
/// Returns the new session id, or `None` when activation failed for any reason (missing
/// entry point, non-S_OK activate result, COM error). Never panics, never throws.
///
/// Every capture mode funnels through here, so there is exactly one implementation of the
/// thread/COM/stop-event/registration dance — the EXCLUDE and INCLUDE paths differ ONLY in
/// the closure they hand in.
fn spawn_session<F>(activate: F, on_chunk: ChunkTsfn) -> Option<u32>
where
    F: FnOnce() -> ActivationResult + Send + 'static,
{
    // Manual-reset stop event the capture loop polls; unsignaled initially. Created here so
    // a concurrent stop can signal it even while the thread is still activating.
    let stop_event = match unsafe { CreateEventW(None, true, false, PCWSTR::null()) } {
        Ok(h) => h,
        Err(_) => return None,
    };

    let (tx, rx): (Sender<bool>, _) = channel();
    let stop_event_for_thread = SendHandle(stop_event);
    let stats = Arc::new(StreamStats::default());
    let stats_for_thread = Arc::clone(&stats);

    let join = std::thread::spawn(move || {
        // Capture the whole SendHandle (Send), not its inner HANDLE field — Rust 2021's
        // disjoint closure capture would otherwise grab the non-Send `.0` directly.
        let stop_handle = stop_event_for_thread;

        // COM on the capture thread (MTA — no message pump needed for WASAPI capture). MTA is
        // also what spares the async-activation completion handler from needing IAgileObject:
        // it never has to marshal into an STA.
        let com = unsafe { CoInitializeEx(None, COINIT_MULTITHREADED) };
        let com_ok = com.is_ok();

        // Success is reported only once the stream is actually running — activation, Initialize,
        // SetEventHandle, GetService and Start all succeeded — then the capture loop takes over.
        let stream = match activate() {
            ActivationResult::Activated(client) => unsafe { start_stream(client) },
            ActivationResult::Unsupported => None,
        };
        match stream {
            Some(stream) => {
                stats_for_thread.update(|s| s.running = true);
                let _ = tx.send(true);
                unsafe { run_capture_loop(stream, on_chunk, stop_handle.0, stats_for_thread) };
            }
            None => {
                let _ = tx.send(false);
            }
        }

        if com_ok {
            unsafe { CoUninitialize() };
        }
    });

    let started = rx
        .recv_timeout(Duration::from_millis(STARTUP_TIMEOUT_MS))
        .unwrap_or(false);
    let session = CaptureSession {
        stop_event,
        join: Some(join),
        stats,
    };

    if !started {
        // Failed, or hung past the startup bound. teardown_session signals the thread (a late
        // start then exits at its first stop poll), joins with a bound, and closes the stop
        // event only if the thread is provably gone — otherwise it is leaked, never recycled.
        teardown_session(session);
        return None;
    }

    let id = NEXT_SESSION_ID.fetch_add(1, Ordering::Relaxed);
    SESSIONS
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .insert(id, session);
    Some(id)
}

// ─── napi surface ───────────────────────────────────────────────────────────────────

/// Begin process-tree EXCLUDE loopback capture, excluding the tree rooted at
/// `exclude_root_pid` (pass the Electron MAIN process PID — EXCLUDE_TARGET_PROCESS_TREE
/// covers the whole tree incl. the separate "Audio Service" utility child). `on_chunk`
/// receives 480-frame (3840-byte) interleaved-stereo f32 buffers, ~one per 10 ms.
///
/// This is the "share a whole screen" mode, and it is mechanically what Discord does for a
/// `screen:*` source (its renderer sends the sentinel `soundsharePid = 1`).
///
/// Returns the session id, or `0` when process-loopback is unavailable on this build OR
/// activation failed — the caller then falls back to Electron "loopback". Never throws.
#[napi(js_name = "startExcludeProcessTree")]
pub fn start_exclude_process_tree(exclude_root_pid: u32, on_chunk: ChunkTsfn) -> napi::Result<u32> {
    Ok(spawn_session(
        move || unsafe {
            activate_process_tree(
                PROCESS_LOOPBACK_MODE_EXCLUDE_TARGET_PROCESS_TREE,
                exclude_root_pid,
            )
        },
        on_chunk,
    )
    .unwrap_or(0))
}

/// Begin app-exclusive INCLUDE loopback capture of the process tree rooted at `target_pid`
/// — captures ONLY that app + its children, so it is self-free AND virtual-audio-device-free
/// by construction (an allowlist cannot pick up a VAC that is not in it). This is the mode
/// Discord uses for a `window:*` source, and the one that fixes the VAC echo.
///
/// Emits the SAME fixed 48k/stereo/f32 480-frame (3840-byte) chunk contract as the EXCLUDE
/// path; the only divergence is the loopback-mode constant.
///
/// Call it once per selected app — sessions are independent and run concurrently, so N apps
/// means N sessions mixed downstream.
///
/// Returns the session id, or `0` on failure. Never throws.
#[napi(js_name = "startIncludeProcessTree")]
pub fn start_include_process_tree(target_pid: u32, on_chunk: ChunkTsfn) -> napi::Result<u32> {
    Ok(spawn_session(
        move || unsafe {
            activate_process_tree(PROCESS_LOOPBACK_MODE_INCLUDE_TARGET_PROCESS_TREE, target_pid)
        },
        on_chunk,
    )
    .unwrap_or(0))
}

/// Stop ONE capture session by id. Signals its capture thread to exit, then joins with a
/// bounded timeout so a hung native teardown can't wedge the caller (composes with the JS
/// wrapper's before-quit Promise.race). Idempotent — an unknown id is a no-op.
#[napi(js_name = "stopSession")]
pub fn stop_session(id: u32) {
    let session = SESSIONS
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .remove(&id);
    if let Some(session) = session {
        teardown_session(session);
    }
}

/// Engine-reported timing for a session — one coherent snapshot (see `TimingSnapshot`).
/// Counters are `f64` because napi has no u64 and these stay exact well past any realistic
/// session length (2^53 frames is ~5900 years at 48k).
#[napi(object)]
pub struct CaptureStats {
    /// Raw DevicePosition of the timing packet. May stay 0 on process loopback — check
    /// `device_position_advances` before using it.
    pub device_position: f64,
    /// Performance counter at which the engine recorded the timing packet, in 100 ns units.
    pub qpc_position100ns: f64,
    /// Packets pulled from the engine.
    pub packets: f64,
    /// Chunks the FFI queue refused (QueueFull). Non-zero means chunk-arrival timing is a lie.
    pub dropped_chunks: f64,
    /// Packets flagged AUDCLNT_BUFFERFLAGS_DATA_DISCONTINUITY.
    pub discontinuities: f64,
    /// Packets whose reported position was flagged invalid.
    pub timestamp_errors: f64,
    /// Bumped on every update; unchanged across polls ⇒ nothing new was published.
    pub sequence: f64,
    /// False once the capture thread has exited.
    pub running: bool,
    /// HRESULT that ended the stream (0 if running or stopped on request).
    pub error_hresult: i32,
    /// Total frames read from the engine (silent packets included) — monotonic.
    pub captured_frames: f64,
    pub last_packet_frames: u32,
    pub silent_packets: f64,
    /// Chunks accepted by the FFI queue.
    pub delivered_chunks: f64,
    /// Whether `timing_frame`/`qpc_position100ns` hold a valid pair yet.
    pub timing_valid: bool,
    /// `captured_frames` index of the timing packet's first frame (pairs with the QPC).
    pub timing_frame: f64,
    /// Valid packets whose DevicePosition moved forward. 0 ⇒ DevicePosition is unusable.
    pub device_position_advances: f64,
}

impl From<TimingSnapshot> for CaptureStats {
    fn from(s: TimingSnapshot) -> Self {
        CaptureStats {
            device_position: s.timing_device_position as f64,
            qpc_position100ns: s.timing_qpc_100ns as f64,
            packets: s.packets as f64,
            dropped_chunks: s.dropped_chunks as f64,
            discontinuities: s.discontinuities as f64,
            timestamp_errors: s.timestamp_errors as f64,
            sequence: s.sequence as f64,
            running: s.running,
            error_hresult: s.error_hresult,
            captured_frames: s.captured_frames as f64,
            last_packet_frames: s.last_packet_frames,
            silent_packets: s.silent_packets as f64,
            delivered_chunks: s.delivered_chunks as f64,
            timing_valid: s.timing_valid,
            timing_frame: s.timing_frame as f64,
            device_position_advances: s.device_position_advances as f64,
        }
    }
}

/// Read a session's timing snapshot. `None` when the id is unknown (never started or stopped).
///
/// Regressing `timing_frame` against `qpc_position100ns` across polls gives the stream's rate
/// against the system performance counter — distinguishing a real clock difference from packets
/// arriving unevenly. Discard windows where `discontinuities` or `timestamp_errors` moved.
#[napi(js_name = "getCaptureStats")]
pub fn get_capture_stats(id: u32) -> Option<CaptureStats> {
    let guard = SESSIONS.lock().unwrap_or_else(|p| p.into_inner());
    Some(guard.get(&id)?.stats.snapshot().into())
}

/// Plain endpoint loopback on one render device — no process filter of any kind.
///
/// Diagnostic surface, not a product path. Endpoint-bound capture carrying an EXCLUDE blob was
/// falsified (the process filter is silently ignored), so this deliberately does not pretend to
/// filter: it captures the chosen device's whole mix. It exists so endpoint-scoped and
/// process-scoped capture can be compared on the same box.
///
/// `device_id` is an `IMMDevice` id, or `None`/`"default"` for the default console render endpoint.
#[napi(js_name = "startRenderEndpointLoopback")]
pub fn start_render_endpoint_loopback(
    device_id: Option<String>,
    on_chunk: ChunkTsfn,
) -> napi::Result<u32> {
    Ok(
        spawn_session(
            move || unsafe { activate_render_endpoint_loopback(device_id.as_deref()) },
            on_chunk,
        )
        .unwrap_or(0),
    )
}

/// Resolve an `IMMDevice` and activate plain loopback capture on it.
unsafe fn activate_render_endpoint_loopback(device_id: Option<&str>) -> ActivationResult {
    let enumerator: IMMDeviceEnumerator =
        match CoCreateInstance(&MMDeviceEnumerator, None, CLSCTX_ALL) {
            Ok(e) => e,
            Err(_) => return ActivationResult::Unsupported,
        };

    let device: IMMDevice = match device_id {
        None | Some("default") => match enumerator.GetDefaultAudioEndpoint(eRender, eConsole) {
            Ok(d) => d,
            Err(_) => return ActivationResult::Unsupported,
        },
        Some(id) => {
            let wide: Vec<u16> = id.encode_utf16().chain(std::iter::once(0)).collect();
            match enumerator.GetDevice(PCWSTR(wide.as_ptr())) {
                Ok(d) => d,
                Err(_) => return ActivationResult::Unsupported,
            }
        }
    };

    // No activation params: a render endpoint activated with a null blob is ordinary loopback.
    let audio_client: IAudioClient = match device.Activate(CLSCTX_ALL, None) {
        Ok(c) => c,
        Err(_) => return ActivationResult::Unsupported,
    };

    if initialize_loopback_client(&audio_client).is_err() {
        return ActivationResult::Unsupported;
    }

    ActivationResult::Activated(audio_client)
}

/// Stop every live capture session. Used for share teardown and the before-quit path.
/// Idempotent — a no-op when nothing is running.
#[napi(js_name = "stopAll")]
pub fn stop_all() {
    let sessions: Vec<CaptureSession> = {
        let mut guard = SESSIONS.lock().unwrap_or_else(|p| p.into_inner());
        guard.drain().map(|(_, s)| s).collect()
    };
    for session in sessions {
        teardown_session(session);
    }
}

/// Signal + bounded join for one session.
///
/// HANDLE-LIFETIME NOTE (load-bearing): on the timeout path we deliberately LEAK the stop
/// event instead of closing it. The capture thread is still alive and still polling that
/// handle; closing it would (a) make its `WaitForSingleObject` return WAIT_FAILED and (b)
/// free the handle value for recycling, so a later `CreateEventW` could hand the orphaned
/// thread an unrelated kernel object to wait on. One leaked event per hung teardown is far
/// cheaper than a thread waiting on someone else's object. (The capture loop also now exits
/// on any non-WAIT_TIMEOUT result, so a leaked-but-signaled event still ends it promptly.)
fn teardown_session(mut session: CaptureSession) {
    unsafe {
        let _ = SetEvent(session.stop_event);
    }

    let mut joined = false;
    if let Some(join) = session.join.take() {
        let deadline =
            std::time::Instant::now() + std::time::Duration::from_millis(STOP_JOIN_TIMEOUT_MS);
        loop {
            if join.is_finished() {
                let _ = join.join();
                joined = true;
                break;
            }
            if std::time::Instant::now() >= deadline {
                // Detach rather than block the caller. Handle intentionally NOT closed.
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
    } else {
        joined = true;
    }

    if joined {
        // Only safe once the thread is provably gone.
        unsafe {
            let _ = CloseHandle(session.stop_event);
        }
    }
}

// ─── Audio-session app enumeration (listAudioApps) ──────────────────────────────────
//
// Clean-room from the public Microsoft Core Audio session APIs (IMMDeviceEnumerator +
// IAudioSessionManager2/IAudioSessionControl2): enumerate every ACTIVE eRender endpoint,
// walk each endpoint's audio sessions, and surface one entry per audio-emitting app PID.
// The picker (Plan 02) lists these so the user can choose an app to INCLUDE-capture.
// Enumeration is fail-closed: ANY failure at ANY step yields an empty list — never an
// error, never a panic across the FFI boundary.

/// One audio-emitting app, as surfaced to the screenshare picker. napi maps the fields to
/// the JS shape `{ processId, displayName, binary }` — the exact contract the existing
/// renderer checklist already consumes. Do NOT widen it.
#[napi(object)]
pub struct AudioAppInfo {
    /// The app's process id (the INCLUDE-capture target).
    pub process_id: u32,
    /// Friendly name: the audio-session display name, else the executable basename.
    pub display_name: String,
    /// The executable basename (e.g. `chrome.exe`).
    pub binary: String,
}

/// Enumerate audio-emitting apps (one entry per PID). Deduped by PID; the system-sounds
/// pseudo-session and GoofCord's OWN process id are dropped. Fail-closed: returns an empty
/// vec on any failure — never throws.
///
/// The Electron "Audio Service" utility CHILD pid is dropped on the TS side in Plan 02
/// (only the main process can resolve it via `app.getAppMetrics()`); this addon drops only
/// its own process id here — the split is intentional.
#[napi(js_name = "listAudioApps")]
pub fn list_audio_apps() -> napi::Result<Vec<AudioAppInfo>> {
    // Session enumeration is COM-apartment-affine: run it on a dedicated MTA thread (the
    // same apartment discipline as the capture thread) so it never depends on — or
    // disturbs — the caller's apartment. A panic on that thread collapses to an empty vec.
    let apps = std::thread::spawn(|| unsafe { enumerate_audio_apps() })
        .join()
        .unwrap_or_default();
    Ok(apps)
}

/// MTA-apartment bracket around the enumeration: CoInitialize the dedicated thread, collect,
/// CoUninitialize — mirrors the capture thread's `CoInitializeEx(COINIT_MULTITHREADED)` +
/// `CoUninitialize` pattern. All COM interfaces are created and dropped inside
/// `collect_audio_apps` before the apartment is torn down.
unsafe fn enumerate_audio_apps() -> Vec<AudioAppInfo> {
    let com = CoInitializeEx(None, COINIT_MULTITHREADED);
    let com_ok = com.is_ok();
    let apps = collect_audio_apps();
    if com_ok {
        CoUninitialize();
    }
    apps
}

/// The enumeration body. Every fallible COM step degrades to "skip this item" or an early
/// empty return — no `?`, no panic, no throw (fail-closed enumeration).
unsafe fn collect_audio_apps() -> Vec<AudioAppInfo> {
    let mut out: Vec<AudioAppInfo> = Vec::new();
    let mut seen: Vec<u32> = Vec::new(); // dedupe by PID (N is tiny; a linear scan is fine).
    let own_pid = GetCurrentProcessId();

    let enumerator: IMMDeviceEnumerator =
        match CoCreateInstance(&MMDeviceEnumerator, None, CLSCTX_ALL) {
            Ok(e) => e,
            Err(_) => return out,
        };

    // Every ACTIVE render endpoint — an app can be emitting on any of them.
    let devices: IMMDeviceCollection =
        match enumerator.EnumAudioEndpoints(eRender, DEVICE_STATE_ACTIVE) {
            Ok(d) => d,
            Err(_) => return out,
        };

    let device_count = devices.GetCount().unwrap_or(0);
    for d in 0..device_count {
        let device: IMMDevice = match devices.Item(d) {
            Ok(dev) => dev,
            Err(_) => continue,
        };
        let manager: IAudioSessionManager2 = match device.Activate(CLSCTX_ALL, None) {
            Ok(m) => m,
            Err(_) => continue,
        };
        let sessions: IAudioSessionEnumerator = match manager.GetSessionEnumerator() {
            Ok(s) => s,
            Err(_) => continue,
        };

        let session_count = sessions.GetCount().unwrap_or(0);
        for s in 0..session_count {
            let control: IAudioSessionControl = match sessions.GetSession(s) {
                Ok(c) => c,
                Err(_) => continue,
            };
            let control2: IAudioSessionControl2 = match control.cast() {
                Ok(c) => c,
                Err(_) => continue,
            };

            // Drop the system-sounds pseudo-session (S_OK == "is system sounds").
            if control2.IsSystemSoundsSession() == S_OK {
                continue;
            }

            let pid = match control2.GetProcessId() {
                Ok(p) => p,
                Err(_) => continue,
            };
            // Drop the "no single process" sentinel (0), GoofCord's own PID, and dupes.
            if pid == 0 || pid == own_pid || seen.contains(&pid) {
                continue;
            }
            seen.push(pid);

            // Prefer the session display name; fall back to the executable basename.
            let binary = process_basename(pid);
            let display_name =
                session_display_name(&control).unwrap_or_else(|| binary.clone());
            out.push(AudioAppInfo {
                process_id: pid,
                display_name,
                binary,
            });
        }
    }

    out
}

/// The session's friendly display name, or `None` when absent. An empty name — or an
/// unexpanded resource reference (`@%SystemRoot%\...,-101`) that would render as gibberish —
/// is treated as "no name" so the caller falls back to the executable basename. The string
/// GetDisplayName hands back is CoTaskMem-allocated; we free it here regardless of parse.
unsafe fn session_display_name(control: &IAudioSessionControl) -> Option<String> {
    let raw: PWSTR = control.GetDisplayName().ok()?;
    if raw.is_null() {
        return None;
    }
    let owned = raw.to_string().ok();
    CoTaskMemFree(Some(raw.as_ptr() as *const core::ffi::c_void));

    let name = owned?;
    let trimmed = name.trim();
    if trimmed.is_empty() || trimmed.starts_with('@') {
        None
    } else {
        Some(trimmed.to_string())
    }
}

/// The executable basename for `pid` (e.g. `chrome.exe`) via `QueryFullProcessImageNameW`,
/// or an empty string when the process can't be opened/queried. Opens with only
/// PROCESS_QUERY_LIMITED_INFORMATION (resolves across integrity levels without elevation).
unsafe fn process_basename(pid: u32) -> String {
    let handle = match OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, false, pid) {
        Ok(h) => h,
        Err(_) => return String::new(),
    };

    let mut buf = [0u16; 260]; // MAX_PATH
    let mut size = buf.len() as u32;
    let queried = QueryFullProcessImageNameW(
        handle,
        PROCESS_NAME_WIN32,
        PWSTR::from_raw(buf.as_mut_ptr()),
        &mut size,
    )
    .is_ok();
    let _ = CloseHandle(handle);

    if !queried || size == 0 {
        return String::new();
    }

    let full = String::from_utf16_lossy(&buf[..size as usize]);
    // basename: keep only the segment after the last path separator.
    full.rsplit(|c| c == '\\' || c == '/')
        .next()
        .unwrap_or("")
        .to_string()
}

// ─── Render-endpoint enumeration (list_render_endpoints) ────────────────────────────
//
// Clean-room from the public Microsoft Core Audio device APIs (IMMDeviceEnumerator +
// IPropertyStore + PKEY_Device_FriendlyName), mirroring OBS's win-wasapi endpoint
// enumeration: list every ACTIVE eRender endpoint, read its device id + friendly name,
// and tag the eConsole default. The selector (Plan 04) presents these so the user can
// point GoofCord at a clean render bus (e.g. a VAC "Stream" endpoint). Fail-closed: ANY
// failure at ANY step yields an empty list — never an error, never a panic across FFI.

/// One active render endpoint, as surfaced to the source selector. napi maps the fields to
/// the JS shape `{ id, name, isDefault }` (`is_default` → `isDefault`). `id` is the raw
/// `IMMDevice` id (the `GetDevice` key); `name` is the friendly name (or the id when the
/// property store has none); `isDefault` marks the eConsole default render endpoint.
#[napi(object)]
pub struct RenderEndpointInfo {
    /// The endpoint's `IMMDevice` id (the explicit render-endpoint selector key).
    pub id: String,
    /// Friendly name (`PKEY_Device_FriendlyName`), falling back to the id.
    pub name: String,
    /// True for the current eConsole default render endpoint (the `"default"` sentinel target).
    pub is_default: bool,
}

/// Enumerate active render endpoints (one entry per `eRender` `DEVICE_STATE_ACTIVE` device),
/// with the eConsole default tagged. Fail-closed: returns an empty vec on any failure — never
/// throws. Runs on a dedicated MTA thread (same apartment discipline as `listAudioApps` and the
/// capture thread) so it never depends on — or disturbs — the caller's apartment.
#[napi(js_name = "listRenderEndpoints")]
pub fn list_render_endpoints() -> napi::Result<Vec<RenderEndpointInfo>> {
    let endpoints = std::thread::spawn(|| unsafe { enumerate_render_endpoints() })
        .join()
        .unwrap_or_default();
    Ok(endpoints)
}

/// MTA-apartment bracket around the render-endpoint enumeration (mirrors `enumerate_audio_apps`):
/// CoInitialize the dedicated thread, collect, CoUninitialize. All COM interfaces are created and
/// dropped inside `collect_render_endpoints` before the apartment is torn down.
unsafe fn enumerate_render_endpoints() -> Vec<RenderEndpointInfo> {
    let com = CoInitializeEx(None, COINIT_MULTITHREADED);
    let com_ok = com.is_ok();
    let out = collect_render_endpoints();
    if com_ok {
        CoUninitialize();
    }
    out
}

/// The enumeration body. Every fallible COM step degrades to "skip this endpoint" or an early
/// empty return — no `?`, no panic, no throw (fail-closed enumeration).
unsafe fn collect_render_endpoints() -> Vec<RenderEndpointInfo> {
    let mut out: Vec<RenderEndpointInfo> = Vec::new();

    let enumerator: IMMDeviceEnumerator =
        match CoCreateInstance(&MMDeviceEnumerator, None, CLSCTX_ALL) {
            Ok(e) => e,
            Err(_) => return out,
        };

    // Resolve the eConsole default once (best-effort) so each entry can be tagged. A failure
    // here just means nothing is tagged default — enumeration still proceeds.
    let default_id: Option<String> = match enumerator.GetDefaultAudioEndpoint(eRender, eConsole) {
        Ok(d) => immdevice_id(&d),
        Err(_) => None,
    };

    let devices: IMMDeviceCollection =
        match enumerator.EnumAudioEndpoints(eRender, DEVICE_STATE_ACTIVE) {
            Ok(d) => d,
            Err(_) => return out,
        };

    let device_count = devices.GetCount().unwrap_or(0);
    for d in 0..device_count {
        let device: IMMDevice = match devices.Item(d) {
            Ok(dev) => dev,
            Err(_) => continue,
        };
        // The id is the selector key — an endpoint without one is unusable, so skip it.
        let id = match immdevice_id(&device) {
            Some(i) => i,
            None => continue,
        };
        let name = endpoint_friendly_name(&device).unwrap_or_else(|| id.clone());
        let is_default = default_id.as_deref() == Some(id.as_str());
        out.push(RenderEndpointInfo { id, name, is_default });
    }

    out
}

/// The `IMMDevice` id string (the `GetDevice`/selector key), or `None` when absent. `GetId`
/// hands back a CoTaskMem-allocated `PWSTR`; we copy it into an owned `String` and free the
/// original with `CoTaskMemFree` (same ownership discipline as `session_display_name`).
unsafe fn immdevice_id(device: &IMMDevice) -> Option<String> {
    let raw: PWSTR = device.GetId().ok()?;
    if raw.is_null() {
        return None;
    }
    let owned = raw.to_string().ok();
    CoTaskMemFree(Some(raw.as_ptr() as *const core::ffi::c_void));
    owned
}

/// The endpoint's friendly name via the property store (`PKEY_Device_FriendlyName`), or `None`.
/// The string lives inside the returned owning `PROPVARIANT` (`VT_LPWSTR`); we copy it into an
/// owned `String` and let the `PROPVARIANT` drop — its `PropVariantClear` frees the string (we
/// OWN the value returned by `GetValue`, so this is the correct release, NOT `CoTaskMemFree`).
unsafe fn endpoint_friendly_name(device: &IMMDevice) -> Option<String> {
    let store: IPropertyStore = device.OpenPropertyStore(STGM_READ).ok()?;
    let prop = store.GetValue(&PKEY_Device_FriendlyName).ok()?;
    let pv = &prop.Anonymous.Anonymous;
    if pv.vt != VT_LPWSTR {
        return None;
    }
    let raw = pv.Anonymous.pwszVal;
    if raw.is_null() {
        return None;
    }
    let owned = raw.to_string().ok()?;
    let trimmed = owned.trim();
    if trimmed.is_empty() {
        None
    } else {
        Some(trimmed.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn captured_frames_count_every_packet_including_silent_and_untimed() {
        let mut s = TimingSnapshot::default();
        s.record_packet(480, 0, 0, 1_000);
        s.record_packet(441, FLAG_SILENT, 0, 1_100);
        s.record_packet(480, FLAG_TIMESTAMP_ERROR, 0, 0);
        assert_eq!(s.captured_frames, 480 + 441 + 480);
        assert_eq!(s.packets, 3);
        assert_eq!(s.silent_packets, 1);
        assert_eq!(s.timestamp_errors, 1);
        assert_eq!(s.last_packet_frames, 480);
    }

    #[test]
    fn timestamp_error_keeps_the_previous_valid_pair() {
        let mut s = TimingSnapshot::default();
        s.record_packet(480, 0, 0, 5_000);
        s.record_packet(480, FLAG_TIMESTAMP_ERROR, 0, 9_999);
        assert!(s.timing_valid);
        assert_eq!((s.timing_frame, s.timing_qpc_100ns), (0, 5_000));
        s.record_packet(480, 0, 0, 5_200);
        assert_eq!((s.timing_frame, s.timing_qpc_100ns), (960, 5_200));
    }

    #[test]
    fn a_device_position_that_never_moves_is_reported_as_unusable() {
        let mut s = TimingSnapshot::default();
        for i in 0..10 {
            s.record_packet(480, 0, 0, 1_000 + i * 100);
        }
        assert_eq!(s.device_position_advances, 0);
        let mut t = TimingSnapshot::default();
        for i in 0..10u64 {
            t.record_packet(480, 0, i * 480, 1_000 + i * 100);
        }
        assert_eq!(t.device_position_advances, 9);
    }

    #[test]
    fn zero_qpc_is_not_a_timing_sample_and_every_update_bumps_sequence() {
        let mut s = TimingSnapshot::default();
        s.record_packet(480, 0, 0, 0);
        assert!(!s.timing_valid);
        s.record_chunk(true);
        s.record_chunk(false);
        s.finish(Some(E_FAIL));
        assert_eq!(s.sequence, 4);
        assert_eq!((s.delivered_chunks, s.dropped_chunks), (1, 1));
        assert!(!s.running);
        assert_eq!(s.error_hresult, E_FAIL.0);
    }
}
