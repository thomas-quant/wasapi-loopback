//! The paired-capture engine: packet timeline → generation → verified offset → `C − R`.
//!
//! Two legs arrive as WASAPI packets: the endpoint loopback `C` (what the user hears) and the
//! process-loopback INCLUDE reference `R` (GoofCord's own playback, device-agnostic). Each leg has
//! its own *local* frame index — frames actually read, silent packets included. Process loopback's
//! DevicePosition is always zero and the endpoint's absolute position is irrelevant, so neither is
//! used. Per packet we keep the local index of its first frame and the engine QPC (else the
//! host's read time) stamped at that same packet start.
//!
//! A *generation* is a stretch in which both legs are gap-free, so a single integer offset
//! `j = i + offset` maps endpoint frame `i` to reference frame `j` for the whole generation. A data
//! discontinuity, or a packet whose start time disagrees with the previous packet on its leg,
//! ends the generation: missing packets are never treated as silence.
//!
//! Per generation:
//! 1. packet timing gives a coarse offset (never trusted to be exact);
//! 2. a bounded correlation search around it nominates an integer offset from natural own audio;
//! 3. the candidate is checked on *later, disjoint* stereo data at unity gain — per channel, the gain
//!    ratio must be provably within `gain_tolerance` of 1 and the fractional-delay projection
//!    within `delay_tolerance` of 0. Only then does the generation lock;
//! 4. while locked every endpoint frame is emitted as `C[i] − R[i + offset]` (unity, no filter, no
//!    adaptation), and a monitor keeps re-checking gain/delay; a conclusive mismatch is a fault.
//!
//! Until a lock, frames are muted (zeros). The one exception: once a lock has *proved* the tap
//! bias between packet timing and content, later generations may pass the endpoint through while
//! the reference is exactly zero over the whole timing-uncertainty window around the mapped frame.

use std::collections::VecDeque;
use std::fmt;

use crate::align::{AlignJob, AlignOutcome, AlignParams, AlignResult};
use crate::ring::FrameRing;
use crate::stats::{Evidence, Verdict};

pub const SAMPLE_RATE: u32 = 48_000;
pub const CHANNELS: usize = 2;
pub const CHUNK_FRAMES: usize = 480;
const HNS_PER_SEC: f64 = 10_000_000.0;

/// All engine limits. Every buffer is bounded by one of these.
#[derive(Clone, Debug)]
pub struct Config {
    /// Per-leg history ring (frames).
    pub history_frames: usize,
    /// Endpoint frames allowed to wait for the reference before the session faults (overflow).
    pub max_pending_frames: usize,
    /// Correlation search half-width around the timing estimate (frames).
    pub search_radius: usize,
    /// Correlation block length (frames).
    pub calib_frames: usize,
    /// Verification / monitor sub-block length (frames).
    pub sub_block_frames: usize,
    /// Sub-blocks per channel before a gain/delay verdict is allowed.
    pub min_sub_blocks: usize,
    /// A candidate still inconclusive after this many valid held-out sub-blocks is dropped.
    pub max_verify_sub_blocks: usize,
    /// Allowed |gain − 1| for the unity contract.
    pub gain_tolerance: f64,
    /// Allowed |fractional delay| (samples).
    pub delay_tolerance: f64,
    /// Interval half-width multiplier for accepting a candidate.
    pub verify_z: f64,
    /// Interval half-width multiplier for declaring loss of lock (stricter: runs forever).
    pub monitor_z: f64,
    /// Valid sub-blocks per (non-overlapping) monitor window.
    pub monitor_window: usize,
    /// Consecutive rejecting monitor windows before the lock is declared lost.
    pub monitor_consecutive: u32,
    /// A reference channel below this RMS in a sub-block contributes no evidence.
    pub min_ref_rms: f64,
    pub ambiguity_ratio: f64,
    pub min_peak_correlation: f64,
    pub lobe_guard: usize,
    pub edge_guard: usize,
    /// Packet-start disagreement that counts as a gap, engine QPC (100 ns).
    pub gap_tolerance_engine_100ns: u64,
    /// Same, when only the host read time is available (much coarser).
    pub gap_tolerance_read_100ns: u64,
    /// Extra frames added to the silent-reference passthrough window.
    pub passthrough_margin_frames: usize,
    /// Own-audio frames spent aligning without a lock before the session faults.
    pub align_timeout_frames: u64,
    /// Endpoint frames in a generation with no reference packet at all before faulting.
    pub reference_absent_frames: u64,
    /// Output chunks held for the host (oldest dropped past this).
    pub max_output_chunks: usize,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            history_frames: 1 << 17, // ~2.7 s per leg
            max_pending_frames: 24_000,
            search_radius: 12_000,
            calib_frames: 24_000,
            sub_block_frames: 2_400,
            min_sub_blocks: 8,
            max_verify_sub_blocks: 200,
            gain_tolerance: 0.03,
            delay_tolerance: 0.05,
            verify_z: 2.0,
            monitor_z: 4.0,
            monitor_window: 20,
            monitor_consecutive: 2,
            min_ref_rms: 1e-3,
            ambiguity_ratio: 0.9,
            min_peak_correlation: 0.05,
            lobe_guard: 32,
            edge_guard: 64,
            gap_tolerance_engine_100ns: 50_000,
            gap_tolerance_read_100ns: 400_000,
            passthrough_margin_frames: 480,
            align_timeout_frames: 20 * SAMPLE_RATE as u64,
            reference_absent_frames: 2 * SAMPLE_RATE as u64,
            max_output_chunks: 64,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Leg {
    Endpoint,
    Reference,
}

impl Leg {
    fn name(self) -> &'static str {
        match self {
            Leg::Endpoint => "endpoint",
            Leg::Reference => "reference",
        }
    }
}

/// Everything WASAPI says about one packet, captured at `GetBuffer` time.
#[derive(Clone, Copy, Debug, Default)]
pub struct PacketInfo {
    pub frames: u32,
    /// `AUDCLNT_BUFFERFLAGS_SILENT`: valid zero samples.
    pub silent: bool,
    /// `AUDCLNT_BUFFERFLAGS_DATA_DISCONTINUITY`: frames were lost before this packet.
    pub discontinuity: bool,
    /// Engine QPC of the packet's first frame (100 ns); `None` on `TIMESTAMP_ERROR` or 0.
    pub engine_qpc_100ns: Option<u64>,
    /// Host QPC when the packet was read (100 ns, same base).
    pub read_qpc_100ns: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Fault(pub String);

impl fmt::Display for Fault {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Aligning,
    Running,
    Failed,
}

impl Phase {
    pub fn as_str(self) -> &'static str {
        match self {
            Phase::Aligning => "aligning",
            Phase::Running => "running",
            Phase::Failed => "failed",
        }
    }
}

#[derive(Clone, Debug)]
pub struct Status {
    pub phase: Phase,
    pub reason: String,
    /// Verified offset (reference index − endpoint index) while running.
    pub offset_frames: Option<i64>,
    /// Endpoint frames captured but not yet emitted.
    pub buffered_frames: u64,
    pub generation: u32,
    pub coarse_offset_frames: Option<f64>,
    pub candidate_offset_frames: Option<i64>,
    /// Latest per-channel unity-gain estimate (verification or monitor).
    pub gain: [Option<f64>; 2],
    /// Latest per-channel fractional-delay estimate (samples).
    pub fractional_delay: [Option<f64>; 2],
    pub endpoint_frames: u64,
    pub reference_frames: u64,
    pub subtracted_frames: u64,
    pub passthrough_frames: u64,
    pub muted_frames: u64,
    pub locks: u32,
    pub rejected_candidates: u32,
    pub discontinuities: u64,
    pub timeline_gaps: u64,
    pub dropped_chunks: u64,
}

impl Default for Status {
    fn default() -> Self {
        Status {
            phase: Phase::Aligning,
            reason: "starting".into(),
            offset_frames: None,
            buffered_frames: 0,
            generation: 0,
            coarse_offset_frames: None,
            candidate_offset_frames: None,
            gain: [None; 2],
            fractional_delay: [None; 2],
            endpoint_frames: 0,
            reference_frames: 0,
            subtracted_frames: 0,
            passthrough_frames: 0,
            muted_frames: 0,
            locks: 0,
            rejected_candidates: 0,
            discontinuities: 0,
            timeline_gaps: 0,
            dropped_chunks: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TimeKind {
    Engine,
    Read,
}

#[derive(Clone, Copy, Debug)]
struct Stamp {
    index: u64,
    t: u64,
    kind: TimeKind,
}

struct LegState {
    ring: FrameRing,
    /// First packet of the current generation.
    anchor: Option<Stamp>,
    /// Previous packet (any generation), for gap detection.
    last: Option<Stamp>,
    gen_start: u64,
}

impl LegState {
    fn new(cap: usize) -> Self {
        LegState { ring: FrameRing::new(cap), anchor: None, last: None, gen_start: 0 }
    }
}

#[derive(Clone, Copy, Debug)]
struct ProvenBias {
    /// Verified offset minus the timing estimate, frames.
    bias: f64,
    kinds: (TimeKind, TimeKind),
}

#[derive(Clone)]
struct StereoEvidence {
    gain: [Evidence; 2],
    delay: [Evidence; 2],
}

impl StereoEvidence {
    fn new(cap: usize) -> Self {
        StereoEvidence {
            gain: [Evidence::new(cap), Evidence::new(cap)],
            delay: [Evidence::new(cap), Evidence::new(cap)],
        }
    }

    fn clear(&mut self) {
        self.gain.iter_mut().chain(self.delay.iter_mut()).for_each(Evidence::clear);
    }

    fn push(&mut self, fit: &[ChannelFit; 2]) -> bool {
        let mut any = false;
        for ((f, gain), delay) in fit.iter().zip(&mut self.gain).zip(&mut self.delay) {
            if let Some(g) = f.gain {
                gain.push(g);
                any = true;
            }
            if let Some(d) = f.delay {
                delay.push(d);
            }
        }
        any
    }

    /// Accept only when every channel with enough evidence accepts both tests; reject if any test
    /// in any channel rejects.
    fn verdict(&self, cfg: &Config, z: f64) -> Verdict {
        let mut decided = 0;
        let mut pending = false;
        for ch in 0..2 {
            let g = self.gain[ch].verdict(1.0, cfg.gain_tolerance, z, cfg.min_sub_blocks);
            let d = self.delay[ch].verdict(0.0, cfg.delay_tolerance, z, cfg.min_sub_blocks);
            if g == Verdict::Reject || d == Verdict::Reject {
                return Verdict::Reject;
            }
            if self.gain[ch].len() >= cfg.min_sub_blocks {
                if g == Verdict::Accept && d == Verdict::Accept {
                    decided += 1;
                } else {
                    pending = true;
                }
            }
        }
        if decided > 0 && !pending {
            Verdict::Accept
        } else {
            Verdict::Inconclusive
        }
    }

    fn describe(&self) -> String {
        let f = |e: &Evidence| e.mean().map_or("n/a".to_string(), |m| format!("{m:.4}"));
        format!(
            "gain L {} R {}, delay L {} R {}",
            f(&self.gain[0]),
            f(&self.gain[1]),
            f(&self.delay[0]),
            f(&self.delay[1])
        )
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct ChannelFit {
    gain: Option<f64>,
    delay: Option<f64>,
}

struct Candidate {
    offset: i64,
    next: u64,
    valid: usize,
    ev: StereoEvidence,
}

pub struct Engine {
    cfg: Config,
    ep: LegState,
    rf: LegState,
    status: Status,
    coarse: Option<f64>,
    proven: Option<ProvenBias>,
    offset: Option<i64>,
    candidate: Option<Candidate>,
    outbox: Option<AlignJob>,
    in_flight: Option<u64>,
    next_job_id: u64,
    next_calib: u64,
    active_unlocked: u64,
    /// Conclusive evidence, this generation, that the endpoint does not carry the reference.
    mismatches: u32,
    last_reject: Option<String>,
    out_next: u64,
    monitor_next: u64,
    monitor: StereoEvidence,
    monitor_fresh: usize,
    monitor_rejects: u32,
    acc: Vec<f32>,
    chunks: VecDeque<Vec<f32>>,
}

impl Engine {
    pub fn new(cfg: Config) -> Self {
        assert!(cfg.max_pending_frames < cfg.history_frames);
        assert!(cfg.calib_frames + 2 * cfg.search_radius < cfg.history_frames);
        let window = cfg.monitor_window;
        Engine {
            ep: LegState::new(cfg.history_frames),
            rf: LegState::new(cfg.history_frames),
            status: Status { reason: "waiting for both capture legs".into(), ..Status::default() },
            coarse: None,
            proven: None,
            offset: None,
            candidate: None,
            outbox: None,
            in_flight: None,
            next_job_id: 1,
            next_calib: 0,
            active_unlocked: 0,
            mismatches: 0,
            last_reject: None,
            out_next: 0,
            monitor_next: 0,
            monitor: StereoEvidence::new(window),
            monitor_fresh: 0,
            monitor_rejects: 0,
            acc: Vec::with_capacity(CHUNK_FRAMES * CHANNELS),
            chunks: VecDeque::new(),
            cfg,
        }
    }

    pub fn config(&self) -> &Config {
        &self.cfg
    }

    pub fn status(&self) -> Status {
        let mut s = self.status.clone();
        s.offset_frames = if s.phase == Phase::Running { self.offset } else { None };
        s.buffered_frames = self.ep.ring.end() - self.out_next;
        s.coarse_offset_frames = self.coarse;
        s.candidate_offset_frames = self.candidate.as_ref().map(|c| c.offset);
        s
    }

    /// Next 480-frame interleaved stereo chunk, in endpoint order: output frame `k` is always
    /// endpoint local frame `k` (subtracted, passed through, or muted).
    pub fn pop_chunk(&mut self) -> Option<Vec<f32>> {
        self.chunks.pop_front()
    }

    /// Alignment work for the host to run (`run_align_job`) and return via `complete_job`.
    pub fn take_job(&mut self) -> Option<AlignJob> {
        self.outbox.take()
    }

    /// Mark the session failed for a reason the engine cannot see (route change, COM error…).
    pub fn fail_external(&mut self, why: impl Into<String>) -> Fault {
        self.fail(why.into())
    }

    fn fail(&mut self, why: String) -> Fault {
        if self.status.phase != Phase::Failed {
            self.status.phase = Phase::Failed;
            self.status.reason = why;
            self.outbox = None;
            self.candidate = None;
        }
        Fault(self.status.reason.clone())
    }

    fn alive(&self) -> Result<(), Fault> {
        match self.status.phase {
            Phase::Failed => Err(Fault(self.status.reason.clone())),
            _ => Ok(()),
        }
    }

    fn leg(&mut self, leg: Leg) -> &mut LegState {
        match leg {
            Leg::Endpoint => &mut self.ep,
            Leg::Reference => &mut self.rf,
        }
    }

    fn tolerance(&self, kind: TimeKind) -> u64 {
        match kind {
            TimeKind::Engine => self.cfg.gap_tolerance_engine_100ns,
            TimeKind::Read => self.cfg.gap_tolerance_read_100ns,
        }
    }

    fn tolerance_frames(&self, kind: TimeKind) -> i64 {
        (self.tolerance(kind) as f64 * SAMPLE_RATE as f64 / HNS_PER_SEC).ceil() as i64
    }

    /// Feed one packet. `samples` is interleaved stereo f32, required unless `info.silent`.
    pub fn push(&mut self, leg: Leg, info: &PacketInfo, samples: Option<&[f32]>) -> Result<(), Fault> {
        self.alive()?;
        let frames = info.frames as usize;
        if frames == 0 {
            return Ok(());
        }
        let data = if info.silent {
            None
        } else {
            match samples {
                Some(s) if s.len() == frames * CHANNELS => Some(s),
                _ => return Err(self.fail(format!("malformed {} packet", leg.name()))),
            }
        };

        let index = self.leg(leg).ring.end();
        let (t, kind) = match info.engine_qpc_100ns {
            Some(q) => (q, TimeKind::Engine),
            None => (info.read_qpc_100ns, TimeKind::Read),
        };

        if info.discontinuity {
            self.status.discontinuities += 1;
            self.reset_generation(format!("data discontinuity on the {} leg", leg.name()));
        } else if let Some(last) = self.leg(leg).last.filter(|l| l.kind == kind) {
            let expected = last.t as f64 + (index - last.index) as f64 * HNS_PER_SEC / SAMPLE_RATE as f64;
            let dev = t as f64 - expected;
            if dev.abs() > self.tolerance(kind) as f64 {
                self.status.timeline_gaps += 1;
                self.reset_generation(format!(
                    "{} leg timeline jumped {:+.1} ms between packets (missing packets are not silence)",
                    leg.name(),
                    dev / 10_000.0
                ));
            }
        }

        let stamp = Stamp { index, t, kind };
        let st = self.leg(leg);
        st.last = Some(stamp);
        if st.ring.push(data, frames).is_err() {
            return Err(self.fail(format!("non-finite samples on the {} leg", leg.name())));
        }
        if st.anchor.is_none() {
            st.anchor = Some(stamp);
        }
        match leg {
            Leg::Endpoint => self.status.endpoint_frames += frames as u64,
            Leg::Reference => self.status.reference_frames += frames as u64,
        }
        self.advance()
    }

    /// Return a finished alignment job. Results from an older generation are ignored.
    pub fn complete_job(&mut self, result: AlignResult) -> Result<(), Fault> {
        self.alive()?;
        if result.generation != self.status.generation || self.in_flight != Some(result.id) {
            return Ok(());
        }
        self.in_flight = None;
        match result.outcome {
            AlignOutcome::Candidate { offset, peak, runner_up } => {
                self.status.reason = format!(
                    "verifying candidate offset {offset} on held-out audio (correlation {peak:.3}, runner-up {runner_up:.3})"
                );
                self.candidate = Some(Candidate {
                    offset,
                    next: self.next_calib,
                    valid: 0,
                    ev: StereoEvidence::new(self.cfg.max_verify_sub_blocks),
                });
            }
            AlignOutcome::Rejected { reason, mismatch } => {
                self.status.rejected_candidates += 1;
                self.mismatches += u32::from(mismatch);
                self.status.reason = format!("not locked: {reason}");
                self.last_reject = Some(reason);
                self.check_align_timeout()?;
            }
        }
        self.advance()
    }

    fn reset_generation(&mut self, why: String) {
        // Frames still waiting can no longer be proven against a valid mapping: mute them.
        while self.out_next < self.ep.ring.end() {
            self.emit([0.0, 0.0]);
            self.status.muted_frames += 1;
            self.out_next += 1;
        }
        self.status.generation += 1;
        self.status.phase = Phase::Aligning;
        self.status.reason = why;
        self.offset = None;
        self.candidate = None;
        self.outbox = None;
        self.in_flight = None;
        self.coarse = None;
        for st in [&mut self.ep, &mut self.rf] {
            let end = st.ring.end();
            st.ring.reset(end);
            st.anchor = None;
            st.gen_start = end;
        }
        self.next_calib = self.ep.ring.end();
        self.active_unlocked = 0;
        self.mismatches = 0;
        self.monitor.clear();
        self.monitor_fresh = 0;
        self.monitor_rejects = 0;
    }

    fn advance(&mut self) -> Result<(), Fault> {
        if self.coarse.is_none() {
            if let (Some(a), Some(b)) = (self.ep.anchor, self.rf.anchor) {
                self.coarse = Some(
                    (b.index as f64 - a.index as f64)
                        + (a.t as f64 - b.t as f64) * SAMPLE_RATE as f64 / HNS_PER_SEC,
                );
            }
        }

        if self.status.phase == Phase::Aligning {
            if self.rf.anchor.is_none()
                && self.ep.ring.end() - self.ep.gen_start > self.cfg.reference_absent_frames
            {
                return Err(self.fail(format!(
                    "reference leg delivered no packets for {:.1} s of endpoint audio (missing packets are not silence)",
                    self.cfg.reference_absent_frames as f64 / SAMPLE_RATE as f64
                )));
            }
            self.poll_calibration();
            self.poll_verification()?;
        }

        match self.status.phase {
            Phase::Aligning => self.emit_aligning(),
            Phase::Running => {
                self.emit_running()?;
                self.run_monitor()?;
            }
            Phase::Failed => return Err(Fault(self.status.reason.clone())),
        }

        let pending = self.ep.ring.end() - self.out_next;
        if pending > self.cfg.max_pending_frames as u64 {
            return Err(self.fail(format!(
                "overflow: {pending} endpoint frames waiting for reference audio that has not arrived \
                 (cap {}); the reference leg stalled",
                self.cfg.max_pending_frames
            )));
        }
        Ok(())
    }

    fn search_center(&self) -> Option<i64> {
        let c = self.coarse?;
        let bias = self.proven.filter(|p| Some(p.kinds) == self.anchor_kinds()).map_or(0.0, |p| p.bias);
        Some((c + bias).round() as i64)
    }

    fn anchor_kinds(&self) -> Option<(TimeKind, TimeKind)> {
        Some((self.ep.anchor?.kind, self.rf.anchor?.kind))
    }

    fn passthrough_window(&self, kinds: (TimeKind, TimeKind)) -> i64 {
        2 * (self.tolerance_frames(kinds.0) + self.tolerance_frames(kinds.1))
            + self.cfg.passthrough_margin_frames as i64
    }

    fn poll_calibration(&mut self) {
        if self.candidate.is_some() || self.in_flight.is_some() {
            return;
        }
        let Some(center) = self.search_center() else {
            self.status.reason = "waiting for packets on both legs".into();
            return;
        };
        let n = self.cfg.calib_frames as i64;
        let b = self.cfg.search_radius as i64;
        loop {
            let i0 = (self.next_calib as i64)
                .max(self.ep.ring.start() as i64)
                .max(self.rf.ring.start() as i64 - center + b);
            let r0 = i0 + center - b;
            let r1 = r0 + n + 2 * b;
            if i0 + n > self.ep.ring.end() as i64 || r1 > self.rf.ring.end() as i64 {
                return;
            }
            let (i0u, r0u, r1u) = (i0 as u64, r0 as u64, r1 as u64);
            let e = self.rf.ring.energy(r0u, r1u);
            let threshold = n as f64 * self.cfg.min_ref_rms * self.cfg.min_ref_rms;
            if e[0].max(e[1]) < threshold {
                // Nothing to align on here: slide on without spending the timeout.
                self.next_calib = (i0 + n / 2) as u64;
                if !self.status.reason.starts_with("not locked") {
                    self.status.reason = "waiting for own audio to align on (reference silent)".into();
                }
                continue;
            }

            let mut endpoint = Vec::new();
            self.ep.ring.copy_interleaved(i0u, i0u + n as u64, &mut endpoint);
            let mut reference = Vec::new();
            self.rf.ring.copy_interleaved(r0u, r1u, &mut reference);
            let id = self.next_job_id;
            self.next_job_id += 1;
            self.outbox = Some(AlignJob {
                id,
                generation: self.status.generation,
                endpoint_start: i0u,
                base_offset: center - b,
                endpoint,
                reference,
                params: AlignParams {
                    ambiguity_ratio: self.cfg.ambiguity_ratio,
                    min_peak_correlation: self.cfg.min_peak_correlation,
                    lobe_guard: self.cfg.lobe_guard,
                    edge_guard: self.cfg.edge_guard,
                },
            });
            self.in_flight = Some(id);
            self.active_unlocked += n as u64;
            // Held-out verification starts after the calibration block: disjoint data.
            self.next_calib = i0u + n as u64;
            self.status.reason = "searching for the integer offset".into();
            return;
        }
    }

    /// Per-channel unity-gain ratio and fractional-delay projection of `C − R` over one sub-block.
    fn measure(&self, start: u64, len: usize, offset: i64) -> [ChannelFit; 2] {
        let min_energy = len as f64 * self.cfg.min_ref_rms * self.cfg.min_ref_rms;
        let mut fit = [ChannelFit::default(); 2];
        for (ch, out) in fit.iter_mut().enumerate() {
            let (mut a, mut x, mut d, mut y) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
            for n in 0..len as u64 {
                let i = start + n;
                let j = (i as i64 + offset) as u64;
                let c = f64::from(self.ep.ring.frame(i)[ch]);
                let r = f64::from(self.rf.ring.frame(j)[ch]);
                let rp = (f64::from(self.rf.ring.frame(j + 1)[ch]) - f64::from(self.rf.ring.frame(j - 1)[ch])) / 2.0;
                a += r * r;
                x += c * r;
                d += rp * rp;
                y += (c - r) * rp;
            }
            if a >= min_energy {
                out.gain = Some(x / a);
                if d > a * 1e-9 {
                    out.delay = Some(y / d);
                }
            }
        }
        fit
    }

    /// Whether a sub-block at `start` for `offset` is fully inside both rings (with ±1 reference
    /// frames for the derivative). `None` = not yet available, `Some(false)` = history lost.
    fn sub_block_ready(&self, start: u64, len: usize, offset: i64) -> Option<bool> {
        let a = start as i64 + offset - 1;
        let b = start as i64 + offset + len as i64 + 1;
        if start + len as u64 > self.ep.ring.end() || b > self.rf.ring.end() as i64 {
            return None;
        }
        Some(start >= self.ep.ring.start() && a >= self.rf.ring.start() as i64)
    }

    fn poll_verification(&mut self) -> Result<(), Fault> {
        let sb = self.cfg.sub_block_frames;
        while let Some(cand) = self.candidate.as_ref() {
            let (start, offset) = (cand.next, cand.offset);
            match self.sub_block_ready(start, sb, offset) {
                None => return Ok(()),
                Some(false) => {
                    self.drop_candidate("held-out history was lost before verification finished".into(), false);
                    return self.check_align_timeout();
                }
                Some(true) => {}
            }
            let fit = self.measure(start, sb, offset);
            let cand = self.candidate.as_mut().expect("candidate");
            cand.next += sb as u64;
            if cand.ev.push(&fit) {
                cand.valid += 1;
                self.active_unlocked += sb as u64;
            }
            for ch in 0..2 {
                self.status.gain[ch] = cand.ev.gain[ch].mean();
                self.status.fractional_delay[ch] = cand.ev.delay[ch].mean();
            }
            match cand.ev.verdict(&self.cfg, self.cfg.verify_z) {
                Verdict::Accept => return self.lock(),
                Verdict::Reject => {
                    let why = format!("candidate offset {offset} failed the unity check on held-out audio ({})", cand.ev.describe());
                    self.drop_candidate(why, true);
                    return self.check_align_timeout();
                }
                Verdict::Inconclusive if cand.valid >= self.cfg.max_verify_sub_blocks => {
                    let why = format!("candidate offset {offset} stayed inconclusive ({})", cand.ev.describe());
                    self.drop_candidate(why, false);
                    return self.check_align_timeout();
                }
                Verdict::Inconclusive => {}
            }
            self.check_align_timeout()?;
        }
        Ok(())
    }

    fn drop_candidate(&mut self, why: String, mismatch: bool) {
        if let Some(c) = self.candidate.take() {
            self.next_calib = self.next_calib.max(c.next);
        }
        self.status.rejected_candidates += 1;
        self.mismatches += u32::from(mismatch);
        self.status.reason = format!("not locked: {why}");
        self.last_reject = Some(why);
    }

    /// After `align_timeout_frames` of own audio without a lock: fault if the evidence says the
    /// endpoint does not carry the reference; otherwise (ambiguous, drowned out, inconclusive) stay
    /// muted and keep trying — lack of evidence is not a mismatch.
    fn check_align_timeout(&mut self) -> Result<(), Fault> {
        if self.active_unlocked <= self.cfg.align_timeout_frames {
            return Ok(());
        }
        let secs = self.active_unlocked as f64 / SAMPLE_RATE as f64;
        let last = self.last_reject.clone().unwrap_or_else(|| "no candidate".into());
        if self.mismatches >= 2 {
            return Err(self.fail(format!(
                "no unity-gain integer offset could be verified in {secs:.1} s of own audio; the endpoint \
                 does not carry a sample-identical copy of the reference (format/DSP/route mismatch). Last: {last}"
            )));
        }
        self.status.reason = format!(
            "not locked after {secs:.1} s of own audio without conclusive evidence; share audio stays muted. Last: {last}"
        );
        self.active_unlocked = 0;
        Ok(())
    }

    fn lock(&mut self) -> Result<(), Fault> {
        let cand = self.candidate.take().expect("candidate");
        let coarse = self.coarse.expect("coarse offset exists before any job");
        let kinds = self.anchor_kinds().expect("anchors exist before any job");
        let bias = cand.offset as f64 - coarse;
        if let Some(p) = self.proven.filter(|p| p.kinds == kinds) {
            let window = self.passthrough_window(kinds);
            if (bias - p.bias).abs() > window as f64 {
                return Err(self.fail(format!(
                    "content offset moved {:+.0} frames against packet timing between generations \
                     (bound ±{window}); the silent-reference passthrough assumption is violated",
                    bias - p.bias
                )));
            }
        }
        self.proven = Some(ProvenBias { bias, kinds });
        self.offset = Some(cand.offset);
        self.status.phase = Phase::Running;
        self.status.locks += 1;
        self.status.reason = format!(
            "locked at offset {} frames (content vs packet timing {bias:+.0}); {}",
            cand.offset,
            cand.ev.describe()
        );
        self.monitor_next = self.out_next;
        self.monitor.clear();
        self.monitor_fresh = 0;
        self.monitor_rejects = 0;
        Ok(())
    }

    fn emit(&mut self, frame: [f32; 2]) {
        self.acc.extend_from_slice(&frame);
        if self.acc.len() == CHUNK_FRAMES * CHANNELS {
            let chunk = std::mem::replace(&mut self.acc, Vec::with_capacity(CHUNK_FRAMES * CHANNELS));
            self.chunks.push_back(chunk);
            if self.chunks.len() > self.cfg.max_output_chunks {
                self.chunks.pop_front();
                self.status.dropped_chunks += 1;
            }
        }
    }

    fn emit_aligning(&mut self) {
        let window = self
            .proven
            .zip(self.search_center())
            .zip(self.anchor_kinds())
            .filter(|((p, _), kinds)| p.kinds == *kinds)
            .map(|((_, center), kinds)| (center, self.passthrough_window(kinds)));

        while self.out_next < self.ep.ring.end() {
            let i = self.out_next;
            let pass = match window {
                None => false,
                Some((center, ws)) => {
                    let a = i as i64 + center - ws;
                    let b = i as i64 + center + ws + 1;
                    if b > self.rf.ring.end() as i64 {
                        break; // wait until the reference covers the whole window
                    }
                    self.rf.ring.contains(a, b) && self.rf.ring.nonzero_in(a as u64, b as u64) == 0
                }
            };
            if pass {
                let f = self.ep.ring.frame(i);
                self.emit(f);
                self.status.passthrough_frames += 1;
            } else {
                self.emit([0.0, 0.0]);
                self.status.muted_frames += 1;
            }
            self.out_next += 1;
        }
    }

    fn emit_running(&mut self) -> Result<(), Fault> {
        let offset = self.offset.expect("running implies an offset");
        while self.out_next < self.ep.ring.end() {
            let j = self.out_next as i64 + offset;
            if j < self.rf.ring.start() as i64 {
                return Err(self.fail(format!(
                    "reference history overrun: frame {j} needed but the ring starts at {}",
                    self.rf.ring.start()
                )));
            }
            if j >= self.rf.ring.end() as i64 {
                break;
            }
            let c = self.ep.ring.frame(self.out_next);
            let r = self.rf.ring.frame(j as u64);
            self.emit([c[0] - r[0], c[1] - r[1]]);
            self.status.subtracted_frames += 1;
            self.out_next += 1;
        }
        Ok(())
    }

    fn run_monitor(&mut self) -> Result<(), Fault> {
        let offset = self.offset.expect("running implies an offset");
        let sb = self.cfg.sub_block_frames;
        while self.monitor_next + sb as u64 <= self.out_next {
            let start = self.monitor_next;
            match self.sub_block_ready(start, sb, offset) {
                None => return Ok(()),
                Some(false) => {
                    self.monitor_next += sb as u64;
                    continue;
                }
                Some(true) => {}
            }
            let fit = self.measure(start, sb, offset);
            self.monitor_next += sb as u64;
            if !self.monitor.push(&fit) {
                continue;
            }
            for ch in 0..2 {
                self.status.gain[ch] = self.monitor.gain[ch].mean();
                self.status.fractional_delay[ch] = self.monitor.delay[ch].mean();
            }
            self.monitor_fresh += 1;
            if self.monitor_fresh < self.cfg.monitor_window {
                continue;
            }
            // Non-overlapping windows: judge, then start a fresh one.
            let verdict = self.monitor.verdict(&self.cfg, self.cfg.monitor_z);
            let described = self.monitor.describe();
            self.monitor.clear();
            self.monitor_fresh = 0;
            if verdict == Verdict::Reject {
                self.monitor_rejects += 1;
                if self.monitor_rejects >= self.cfg.monitor_consecutive {
                    return Err(self.fail(format!(
                        "lost lock at offset {offset}: the endpoint no longer carries the reference at unity gain ({described})"
                    )));
                }
            } else {
                self.monitor_rejects = 0;
            }
        }
        Ok(())
    }
}
