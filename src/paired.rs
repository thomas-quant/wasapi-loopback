// Endpoint-minus-own-process capture: ONE native owner for both legs.
//
// One thread owns both IAudioClients — plain loopback of one render endpoint (what the user hears,
// device-scoped, so a virtual cable that never reaches it stays out) and process-loopback INCLUDE
// of GoofCord's own tree (the reference). Every packet of both legs goes into the portable
// `wasapi_subtract_core::Engine` with its flags, engine QPC and read time BEFORE any rechunking;
// the engine emits `endpoint − reference` at a verified integer offset and unity gain, passes the
// endpoint through only where the reference is provably silent, and mutes everything else. There
// is no EXCLUDE fallback and no raw-endpoint fallback anywhere in this file. See SUBTRACTION.md.
//
// The correlation search runs on a worker thread (pure Rust, no COM) so a search never stalls the
// capture legs. Every other decision stays on the owner thread.

use std::sync::mpsc::{sync_channel, TryRecvError};
use std::time::Instant;

use wasapi_subtract_core::{
    route, run_align_job, AlignJob, AlignResult, Config, Engine, Leg, PacketInfo, Status,
};
use windows::Win32::Foundation::WAIT_FAILED;
use windows::Win32::Media::Audio::AudioSessionStateActive;
use windows::Win32::System::Diagnostics::ToolHelp::{
    CreateToolhelp32Snapshot, Process32FirstW, Process32NextW, PROCESSENTRY32W, TH32CS_SNAPPROCESS,
};
use windows::Win32::System::Performance::{QueryPerformanceCounter, QueryPerformanceFrequency};
use windows::Win32::System::Threading::WaitForMultipleObjects;

use super::*;

/// Two activations (the process one alone may take ACTIVATION_TIMEOUT_MS) plus the guard snapshot.
const PAIR_STARTUP_TIMEOUT_MS: u64 = ACTIVATION_TIMEOUT_MS as u64 + 3000;

/// How often own-tree sessions are checked against the captured endpoint. Also the worst-case
/// window in which newly started off-endpoint own playback can leak before the session fails.
const ROUTE_GUARD_INTERVAL: Duration = Duration::from_millis(250);

/// Upper bound on one wait for either leg's event; also bounds alignment-result pickup latency.
const PAIR_WAIT_MS: u32 = 20;

/// State shared between a paired session's owner thread and `getSubtractionStatus`.
pub(crate) struct PairShared {
    status: Mutex<Status>,
    endpoint_id: Mutex<String>,
}

impl PairShared {
    fn new() -> Self {
        PairShared {
            status: Mutex::new(Status::default()),
            endpoint_id: Mutex::new(String::new()),
        }
    }

    fn publish(&self, status: Status) {
        *self.status.lock().unwrap_or_else(|p| p.into_inner()) = status;
    }
}

/// Why the most recent `startEndpointMinusSelf` returned 0 (cleared on success).
static LAST_START_ERROR: Mutex<Option<String>> = Mutex::new(None);

/// JSON-serializable subtraction state. `state` is `aligning` (share audio muted, or passed
/// through only where the reference is provably silent), `running` (subtracting at `offsetFrames`)
/// or `failed` (the stream has ended; `reason` says why and the chunk callback got an error).
#[napi(object)]
pub struct SubtractionStatus {
    pub state: String,
    pub reason: String,
    /// Verified reference-index − endpoint-index offset while running, else 0 (see `locked`).
    pub offset_frames: f64,
    pub locked: bool,
    /// Endpoint frames captured but not yet emitted (waiting for the reference).
    pub buffered_frames: f64,
    /// Bumped on every timeline break (discontinuity or gap); each generation re-verifies.
    pub generation: u32,
    pub endpoint_id: String,
    pub coarse_offset_frames: Option<f64>,
    pub candidate_offset_frames: Option<f64>,
    pub gain_left: Option<f64>,
    pub gain_right: Option<f64>,
    pub delay_left: Option<f64>,
    pub delay_right: Option<f64>,
    pub endpoint_frames: f64,
    pub reference_frames: f64,
    pub subtracted_frames: f64,
    pub passthrough_frames: f64,
    pub muted_frames: f64,
    pub locks: u32,
    pub rejected_candidates: u32,
    pub discontinuities: f64,
    pub timeline_gaps: f64,
    pub dropped_chunks: f64,
}

impl SubtractionStatus {
    fn from_shared(shared: &PairShared) -> Self {
        let s = shared.status.lock().unwrap_or_else(|p| p.into_inner()).clone();
        let endpoint_id = shared.endpoint_id.lock().unwrap_or_else(|p| p.into_inner()).clone();
        SubtractionStatus {
            state: s.phase.as_str().to_string(),
            reason: s.reason,
            offset_frames: s.offset_frames.unwrap_or(0) as f64,
            locked: s.offset_frames.is_some(),
            buffered_frames: s.buffered_frames as f64,
            generation: s.generation,
            endpoint_id,
            coarse_offset_frames: s.coarse_offset_frames,
            candidate_offset_frames: s.candidate_offset_frames.map(|v| v as f64),
            gain_left: s.gain[0],
            gain_right: s.gain[1],
            delay_left: s.fractional_delay[0],
            delay_right: s.fractional_delay[1],
            endpoint_frames: s.endpoint_frames as f64,
            reference_frames: s.reference_frames as f64,
            subtracted_frames: s.subtracted_frames as f64,
            passthrough_frames: s.passthrough_frames as f64,
            muted_frames: s.muted_frames as f64,
            locks: s.locks,
            rejected_candidates: s.rejected_candidates,
            discontinuities: s.discontinuities as f64,
            timeline_gaps: s.timeline_gaps as f64,
            dropped_chunks: s.dropped_chunks as f64,
        }
    }
}

// ─── napi surface ───────────────────────────────────────────────────────────────────

/// Capture render endpoint `device_id` (`None`/`null`/`"default"` = eConsole default) MINUS the
/// process-loopback INCLUDE capture of the tree rooted at `root_pid` (the Electron main process).
///
/// Emits the same 480-frame 48 kHz stereo f32 chunks as every other mode. Returns the session id
/// once both legs are running, or `0` when the endpoint format, route or API is unsupported
/// (`getLastSubtractionStartError()` says why). A later fault ends the session and delivers one
/// error through `on_chunk`; `getSubtractionStatus` keeps the reason until `stopSession`.
#[napi(js_name = "startEndpointMinusSelf")]
pub fn start_endpoint_minus_self(
    root_pid: u32,
    device_id: Option<String>,
    on_chunk: ChunkTsfn,
) -> napi::Result<u32> {
    let started = spawn_pair(root_pid, device_id, on_chunk);
    let mut last = LAST_START_ERROR.lock().unwrap_or_else(|p| p.into_inner());
    match started {
        Ok(id) => {
            *last = None;
            Ok(id)
        }
        Err(why) => {
            *last = Some(why);
            Ok(0)
        }
    }
}

/// Subtraction state for a session started by `startEndpointMinusSelf`; `None` for unknown or
/// other-mode sessions.
#[napi(js_name = "getSubtractionStatus")]
pub fn get_subtraction_status(id: u32) -> Option<SubtractionStatus> {
    let guard = SESSIONS.lock().unwrap_or_else(|p| p.into_inner());
    let shared = guard.get(&id)?.subtraction.as_ref()?;
    Some(SubtractionStatus::from_shared(shared))
}

/// The reason the last `startEndpointMinusSelf` returned 0, or `None` if it succeeded.
#[napi(js_name = "getLastSubtractionStartError")]
pub fn get_last_subtraction_start_error() -> Option<String> {
    LAST_START_ERROR.lock().unwrap_or_else(|p| p.into_inner()).clone()
}

// ─── Session spawn ──────────────────────────────────────────────────────────────────

fn spawn_pair(root_pid: u32, device_id: Option<String>, on_chunk: ChunkTsfn) -> Result<u32, String> {
    let stop_event = unsafe { CreateEventW(None, true, false, PCWSTR::null()) }
        .map_err(|e| format!("CreateEventW failed: {e}"))?;
    let (tx, rx) = channel::<Result<(), String>>();
    let stop_for_thread = SendHandle(stop_event);
    let stats = Arc::new(StreamStats::default());
    let stats_for_thread = Arc::clone(&stats);
    let shared = Arc::new(PairShared::new());
    let shared_for_thread = Arc::clone(&shared);

    let join = std::thread::spawn(move || {
        let stop = stop_for_thread;
        let com_ok = unsafe { CoInitializeEx(None, COINIT_MULTITHREADED) }.is_ok();
        match unsafe { open_pair(root_pid, device_id.as_deref()) } {
            Ok(pair) => {
                *shared_for_thread.endpoint_id.lock().unwrap_or_else(|p| p.into_inner()) =
                    pair.endpoint_id.clone();
                stats_for_thread.update(|s| s.running = true);
                let _ = tx.send(Ok(()));
                unsafe {
                    run_pair(pair, root_pid, on_chunk, stop.0, &stats_for_thread, &shared_for_thread)
                };
            }
            Err(why) => {
                let _ = tx.send(Err(why));
            }
        }
        if com_ok {
            unsafe { CoUninitialize() };
        }
    });

    let verdict = rx
        .recv_timeout(Duration::from_millis(PAIR_STARTUP_TIMEOUT_MS))
        .unwrap_or_else(|_| Err("startup timed out".into()));
    let session = CaptureSession {
        stop_event,
        join: Some(join),
        stats,
        subtraction: Some(shared),
    };
    if let Err(why) = verdict {
        teardown_session(session);
        return Err(why);
    }
    let id = NEXT_SESSION_ID.fetch_add(1, Ordering::Relaxed);
    SESSIONS
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .insert(id, session);
    Ok(id)
}

struct PairSetup {
    enumerator: IMMDeviceEnumerator,
    endpoint: RunningStream,
    reference: RunningStream,
    endpoint_id: String,
    follow_default: bool,
}

/// Resolve and vet the endpoint, then start both legs. Any refusal is a start failure with a
/// reason — never a silently narrower or wider capture.
unsafe fn open_pair(root_pid: u32, device_id: Option<&str>) -> Result<PairSetup, String> {
    let enumerator: IMMDeviceEnumerator = CoCreateInstance(&MMDeviceEnumerator, None, CLSCTX_ALL)
        .map_err(|e| format!("MMDeviceEnumerator unavailable: {e}"))?;
    let follow_default = matches!(device_id, None | Some("default"));
    let device: IMMDevice = match device_id {
        None | Some("default") => enumerator
            .GetDefaultAudioEndpoint(eRender, eConsole)
            .map_err(|e| format!("no default render endpoint: {e}"))?,
        Some(id) => {
            let wide: Vec<u16> = id.encode_utf16().chain(std::iter::once(0)).collect();
            enumerator
                .GetDevice(PCWSTR(wide.as_ptr()))
                .map_err(|e| format!("render endpoint {id} not found: {e}"))?
        }
    };
    let endpoint_id = immdevice_id(&device).ok_or("render endpoint has no id")?;

    let client: IAudioClient = device
        .Activate(CLSCTX_ALL, None)
        .map_err(|e| format!("cannot activate endpoint {endpoint_id}: {e}"))?;
    check_endpoint_format(&client)?;
    route_guard(&enumerator, &endpoint_id, root_pid, follow_default)?;
    initialize_loopback_client(&client)
        .map_err(|e| format!("endpoint loopback Initialize failed: {e}"))?;

    let reference_client =
        match activate_process_tree(PROCESS_LOOPBACK_MODE_INCLUDE_TARGET_PROCESS_TREE, root_pid) {
            ActivationResult::Activated(c) => c,
            ActivationResult::Unsupported => {
                return Err("process-loopback INCLUDE of the own tree is unavailable here".into())
            }
        };
    let reference =
        start_stream(reference_client).ok_or("could not start the reference (process INCLUDE) stream")?;
    let Some(endpoint) = start_stream(client) else {
        stop_stream(reference);
        return Err("could not start the endpoint loopback stream".into());
    };
    Ok(PairSetup { enumerator, endpoint, reference, endpoint_id, follow_default })
}

/// Only a native 48 kHz stereo endpoint is accepted. Anything else means the endpoint tap and the
/// process tap each go through their own sample-rate converter / channel matrix, and two
/// independent converters are not sample-identical — that is a rejection, not something to fit.
unsafe fn check_endpoint_format(client: &IAudioClient) -> Result<(), String> {
    let p = client
        .GetMixFormat()
        .map_err(|e| format!("cannot read the endpoint mix format: {e}"))?;
    if p.is_null() {
        return Err("endpoint returned no mix format".into());
    }
    let wf: WAVEFORMATEX = std::ptr::read_unaligned(p);
    let extensible = wf.wFormatTag == WAVE_FORMAT_EXTENSIBLE as u16
        && size_of::<WAVEFORMATEX>() + usize::from(wf.cbSize) >= size_of::<WAVEFORMATEXTENSIBLE>();
    let mask = extensible.then(|| std::ptr::read_unaligned(p as *const WAVEFORMATEXTENSIBLE).dwChannelMask);
    CoTaskMemFree(Some(p as *const core::ffi::c_void));

    let (rate, channels) = (wf.nSamplesPerSec, wf.nChannels);
    if rate != SAMPLE_RATE {
        return Err(format!(
            "endpoint mixes at {rate} Hz; only {SAMPLE_RATE} Hz endpoints are supported (resampled taps are not sample-identical)"
        ));
    }
    if channels != CHANNELS {
        return Err(format!(
            "endpoint has {channels} channels; only stereo endpoints are supported (downmixed taps are not sample-identical)"
        ));
    }
    if let Some(m) = mask.filter(|m| *m != 0 && *m != SPEAKER_STEREO_MASK) {
        return Err(format!("endpoint channel mask 0x{m:X} is not plain front stereo"));
    }
    Ok(())
}

unsafe fn stop_stream(stream: RunningStream) {
    let RunningStream { audio_client, capture, audio_event } = stream;
    let _ = audio_client.Stop();
    drop(capture);
    drop(audio_client);
    let _ = CloseHandle(audio_event);
}

// ─── Route guard ────────────────────────────────────────────────────────────────────

/// Fails when the captured endpoint no longer is where GoofCord plays: the default moved away from
/// a default-following capture, or any own-tree process has an active session on another endpoint.
unsafe fn route_guard(
    enumerator: &IMMDeviceEnumerator,
    endpoint_id: &str,
    root_pid: u32,
    follow_default: bool,
) -> Result<(), String> {
    if follow_default {
        let current = enumerator
            .GetDefaultAudioEndpoint(eRender, eConsole)
            .ok()
            .and_then(|d| immdevice_id(&d));
        match current {
            Some(id) if id.eq_ignore_ascii_case(endpoint_id) => {}
            Some(id) => {
                return Err(format!(
                    "default render endpoint changed ({endpoint_id} -> {id}); restart the share to follow it"
                ))
            }
            None => return Err("the default render endpoint disappeared".into()),
        }
    }
    let own = own_process_tree(root_pid)?;
    let sessions = active_render_sessions(enumerator)?;
    route::check_own_routes(endpoint_id, &own, &sessions)
}

unsafe fn own_process_tree(root_pid: u32) -> Result<Vec<u32>, String> {
    let snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0)
        .map_err(|e| format!("process snapshot failed: {e}"))?;
    let mut entries = Vec::new();
    let mut entry = PROCESSENTRY32W {
        dwSize: size_of::<PROCESSENTRY32W>() as u32,
        ..Default::default()
    };
    let mut more = Process32FirstW(snapshot, &mut entry).is_ok();
    while more {
        entries.push((entry.th32ProcessID, entry.th32ParentProcessID));
        more = Process32NextW(snapshot, &mut entry).is_ok();
    }
    let _ = CloseHandle(snapshot);
    Ok(route::process_tree(root_pid, &entries))
}

/// Every *active* session on every active render endpoint. Unlike `listAudioApps` this is not
/// fail-open: if an endpoint cannot be inspected the guard cannot vouch for the route.
unsafe fn active_render_sessions(
    enumerator: &IMMDeviceEnumerator,
) -> Result<Vec<route::SessionInfo>, String> {
    let devices: IMMDeviceCollection = enumerator
        .EnumAudioEndpoints(eRender, DEVICE_STATE_ACTIVE)
        .map_err(|e| format!("cannot enumerate render endpoints: {e}"))?;
    let count = devices.GetCount().map_err(|e| format!("cannot count render endpoints: {e}"))?;
    let mut out = Vec::new();
    for d in 0..count {
        let Ok(device) = devices.Item(d) else { continue };
        let Some(id) = immdevice_id(&device) else { continue };
        let manager: IAudioSessionManager2 = device
            .Activate(CLSCTX_ALL, None)
            .map_err(|e| format!("cannot inspect sessions on {id}: {e}"))?;
        let sessions: IAudioSessionEnumerator = manager
            .GetSessionEnumerator()
            .map_err(|e| format!("cannot enumerate sessions on {id}: {e}"))?;
        for s in 0..sessions.GetCount().unwrap_or(0) {
            let Ok(control) = sessions.GetSession(s) else { continue };
            if control.GetState().ok() != Some(AudioSessionStateActive) {
                continue;
            }
            let Ok(control2) = control.cast::<IAudioSessionControl2>() else { continue };
            let Ok(pid) = control2.GetProcessId() else { continue };
            out.push(route::SessionInfo { endpoint_id: id.clone(), pid, active: true });
        }
    }
    Ok(out)
}

// ─── Owner loop ─────────────────────────────────────────────────────────────────────

struct QpcClock {
    freq: i64,
}

impl QpcClock {
    fn new() -> Self {
        let mut freq = 0i64;
        unsafe {
            let _ = QueryPerformanceFrequency(&mut freq);
        }
        QpcClock { freq: freq.max(1) }
    }

    /// Now, in the same 100 ns QPC units GetBuffer reports.
    fn now_100ns(&self) -> u64 {
        let mut count = 0i64;
        unsafe {
            let _ = QueryPerformanceCounter(&mut count);
        }
        (i128::from(count) * 10_000_000 / i128::from(self.freq)) as u64
    }
}

enum LegError {
    Hr(HRESULT, &'static str),
    Engine(String),
}

/// Pull every available packet of one leg into the engine, with its flags and timing, before the
/// buffer is released.
unsafe fn drain_leg(
    stream: &RunningStream,
    leg: Leg,
    engine: &mut Engine,
    stats: Option<&StreamStats>,
    clock: &QpcClock,
    scratch: &mut Vec<f32>,
) -> Result<(), LegError> {
    loop {
        let pending = stream
            .capture
            .GetNextPacketSize()
            .map_err(|e| LegError::Hr(e.code(), "GetNextPacketSize"))?;
        if pending == 0 {
            return Ok(());
        }
        let mut data: *mut u8 = std::ptr::null_mut();
        let (mut frames, mut flags, mut device_pos, mut qpc) = (0u32, 0u32, 0u64, 0u64);
        stream
            .capture
            .GetBuffer(&mut data, &mut frames, &mut flags, Some(&mut device_pos), Some(&mut qpc))
            .map_err(|e| LegError::Hr(e.code(), "GetBuffer"))?;
        let read_qpc_100ns = clock.now_100ns();
        if let Some(s) = stats {
            s.update(|t| t.record_packet(frames, flags, device_pos, qpc));
        }
        let info = PacketInfo {
            frames,
            silent: flags & FLAG_SILENT != 0 || data.is_null(),
            discontinuity: flags & FLAG_DATA_DISCONTINUITY != 0,
            engine_qpc_100ns: (flags & FLAG_TIMESTAMP_ERROR == 0 && qpc != 0).then_some(qpc),
            read_qpc_100ns,
        };
        let len = frames as usize * CHANNELS as usize;
        let pushed = if info.silent {
            engine.push(leg, &info, None)
        } else if (data as usize) % std::mem::align_of::<f32>() == 0 {
            engine.push(leg, &info, Some(std::slice::from_raw_parts(data as *const f32, len)))
        } else {
            scratch.clear();
            scratch.extend(
                std::slice::from_raw_parts(data, len * 4)
                    .chunks_exact(4)
                    .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]])),
            );
            engine.push(leg, &info, Some(scratch.as_slice()))
        };
        // The buffer goes back to the engine whatever the verdict was.
        let released = stream.capture.ReleaseBuffer(frames);
        pushed.map_err(|f| LegError::Engine(f.0))?;
        released.map_err(|e| LegError::Hr(e.code(), "ReleaseBuffer"))?;
    }
}

unsafe fn run_pair(
    pair: PairSetup,
    root_pid: u32,
    on_chunk: ChunkTsfn,
    stop_event: HANDLE,
    stats: &StreamStats,
    shared: &PairShared,
) {
    let PairSetup { enumerator, endpoint, reference, endpoint_id, follow_default } = pair;
    let mut engine = Engine::new(Config::default());

    let (job_tx, job_rx) = sync_channel::<AlignJob>(1);
    let (result_tx, result_rx) = channel::<AlignResult>();
    let worker = std::thread::spawn(move || {
        while let Ok(job) = job_rx.recv() {
            if result_tx.send(run_align_job(&job)).is_err() {
                break;
            }
        }
    });

    let clock = QpcClock::new();
    let handles = [stop_event, endpoint.audio_event, reference.audio_event];
    let mut scratch = Vec::new();
    let mut last_guard = Instant::now();
    let mut failure: Option<(String, HRESULT)> = None;

    'run: loop {
        // Same stop discipline as run_capture_loop: anything but "still unsignaled" means exit.
        if WaitForSingleObject(stop_event, 0) != WAIT_TIMEOUT {
            break;
        }
        if WaitForMultipleObjects(&handles, false, PAIR_WAIT_MS) == WAIT_FAILED {
            failure = Some(("waiting on the capture events failed".into(), E_FAIL));
            break;
        }

        for (leg, stream, leg_stats) in [
            (Leg::Endpoint, &endpoint, Some(stats)),
            (Leg::Reference, &reference, None),
        ] {
            match drain_leg(stream, leg, &mut engine, leg_stats, &clock, &mut scratch) {
                Ok(()) => {}
                Err(LegError::Hr(hr, call)) => {
                    let why = format!("{call} failed on the {leg:?} leg: HRESULT 0x{:08X}", hr.0 as u32);
                    failure = Some((why, hr));
                    break 'run;
                }
                Err(LegError::Engine(why)) => {
                    failure = Some((why, E_FAIL));
                    break 'run;
                }
            }
        }

        loop {
            match result_rx.try_recv() {
                Ok(result) => {
                    if let Err(fault) = engine.complete_job(result) {
                        failure = Some((fault.0, E_FAIL));
                        break 'run;
                    }
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    failure = Some(("alignment worker exited".into(), E_FAIL));
                    break 'run;
                }
            }
        }
        if let Some(job) = engine.take_job() {
            // The engine keeps at most one job in flight, so the one-slot channel is never full.
            if job_tx.try_send(job).is_err() {
                failure = Some(("alignment worker unavailable".into(), E_FAIL));
                break;
            }
        }

        while let Some(chunk) = engine.pop_chunk() {
            let bytes: Vec<u8> = chunk.iter().flat_map(|v| v.to_le_bytes()).collect();
            let delivered = on_chunk.call(Ok(bytes.into()), ThreadsafeFunctionCallMode::NonBlocking)
                == napi::Status::Ok;
            stats.update(|s| s.record_chunk(delivered));
        }

        if last_guard.elapsed() >= ROUTE_GUARD_INTERVAL {
            last_guard = Instant::now();
            if let Err(why) = route_guard(&enumerator, &endpoint_id, root_pid, follow_default) {
                failure = Some((why, E_FAIL));
                break;
            }
        }
        shared.publish(engine.status());
    }

    stop_stream(endpoint);
    stop_stream(reference);
    drop(enumerator);
    drop(job_tx);
    let _ = worker.join();

    if let Some((why, _)) = &failure {
        engine.fail_external(why.clone());
    }
    let status = engine.status();
    shared.publish(status.clone());
    stats.update(|s| s.finish(failure.as_ref().map(|(_, hr)| *hr)));
    if failure.is_some() {
        report_failure(&on_chunk, stop_event, &status.reason);
    }
}

/// Deliver the terminal error through the chunk callback. NonBlocking can hit QueueFull behind
/// audio chunks, so retry briefly — but never block, and give up once a stop is requested.
fn report_failure(on_chunk: &ChunkTsfn, stop_event: HANDLE, reason: &str) {
    let message = format!("endpoint-minus-self capture failed: {reason}");
    for _ in 0..50 {
        let status = on_chunk.call(
            Err(napi::Error::new(napi::Status::GenericFailure, message.clone())),
            ThreadsafeFunctionCallMode::NonBlocking,
        );
        if status != napi::Status::QueueFull {
            return;
        }
        if unsafe { WaitForSingleObject(stop_event, 10) } != WAIT_TIMEOUT {
            return;
        }
    }
}
