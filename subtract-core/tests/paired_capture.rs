//! Packet-level simulation of the two capture legs against the engine.
//!
//! Model: a wall clock in frames. The reference leg reports own audio `own(w)` at wall `w`; the
//! endpoint at wall `w` carries `other(w) + gain·own(w + bias)`. Each leg delivers packets of its
//! own sizes, with an engine QPC at the packet start and an arrival latency, and may drop packets
//! (with or without the discontinuity flag), stop, or slip a frame. Output frame `k` is endpoint
//! local frame `k`, whose wall we track, so every output sample can be checked against the truth.

use std::collections::HashMap;

use wasapi_subtract_core::{run_align_job, Config, Engine, Fault, Leg, PacketInfo, Phase, Status};

const F: i64 = 48_000;

fn noise(seed: u64, i: i64) -> f32 {
    let mut z = seed
        .wrapping_mul(0xD1B5_4A32_D192_ED03)
        .wrapping_add((i as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15));
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^= z >> 31;
    (z >> 40) as f32 / (1u64 << 23) as f32 - 1.0
}

/// Mildly coloured and independent per channel (4-tap moving average of white noise).
fn music(seed: u64, w: i64, ch: usize) -> f32 {
    let s = seed * 2 + ch as u64;
    0.2 * (noise(s, w) + noise(s, w - 1) + noise(s, w - 2) + noise(s, w - 3))
}

#[derive(Clone)]
struct LegPlan {
    start: i64,
    sizes: Vec<u32>,
    latency: i64,
    /// Wall ranges whose packets are never delivered.
    drops: Vec<(i64, i64)>,
    /// Flag the first packet after a drop with DATA_DISCONTINUITY.
    flag_drops: bool,
    stop_at: Option<i64>,
    read_time_only: bool,
}

impl LegPlan {
    fn new(start: i64, sizes: &[u32], latency: i64) -> Self {
        LegPlan {
            start,
            sizes: sizes.to_vec(),
            latency,
            drops: vec![],
            flag_drops: false,
            stop_at: None,
            read_time_only: false,
        }
    }
}

type Src = Box<dyn Fn(i64, usize) -> f32>;

struct Scenario {
    own: Src,
    other: Src,
    bias: i64,
    gain: f32,
    swap: bool,
    ep: LegPlan,
    rf: LegPlan,
    end: i64,
    /// Endpoint delivers the frame at this wall twice (content slips, timing stays in tolerance).
    slip_at: Option<i64>,
    cfg: Config,
}

impl Scenario {
    fn new(bias: i64) -> Self {
        Scenario {
            own: Box::new(|w, ch| music(1, w, ch)),
            other: Box::new(|w, ch| music(2, w, ch)),
            bias,
            gain: 1.0,
            swap: false,
            ep: LegPlan::new(7_000, &[480], 240),
            rf: LegPlan::new(0, &[480], 480),
            end: 6 * F,
            slip_at: None,
            cfg: Config::default(),
        }
    }

    fn endpoint_sample(&self, w: i64, ch: usize) -> f32 {
        let och = if self.swap { 1 - ch } else { ch };
        (self.other)(w, ch) + self.gain * (self.own)(w + self.bias, och)
    }
}

struct Pkt {
    leg: Leg,
    arrival: i64,
    order: usize,
    info: PacketInfo,
    samples: Option<Vec<f32>>,
    walls: Vec<i64>,
}

fn hns(w: i64) -> u64 {
    (1_000_000_000 + w * 10_000_000 / F) as u64
}

fn packets(
    plan: &LegPlan,
    leg: Leg,
    end: i64,
    slip: Option<i64>,
    content: &dyn Fn(i64, usize) -> f32,
) -> Vec<Pkt> {
    let mut out = Vec::new();
    let (mut w, mut k, mut flag) = (plan.start, 0usize, false);
    while w < end && plan.stop_at.is_none_or(|s| w < s) {
        let size = plan.sizes[k % plan.sizes.len()] as i64;
        k += 1;
        let start = w;
        w += size;
        if plan.drops.iter().any(|&(a, b)| start < b && start + size > a) {
            flag |= plan.flag_drops;
            continue;
        }
        let mut walls: Vec<i64> = (start..start + size).collect();
        if let Some(s) = slip.filter(|s| (start..start + size).contains(s)) {
            walls.insert((s - start) as usize, s);
        }
        let samples: Vec<f32> = walls.iter().flat_map(|&x| [content(x, 0), content(x, 1)]).collect();
        let silent = samples.iter().all(|v| *v == 0.0);
        let arrival = start + size + plan.latency;
        let info = PacketInfo {
            frames: walls.len() as u32,
            silent,
            discontinuity: flag,
            engine_qpc_100ns: (!plan.read_time_only).then(|| hns(start)),
            read_qpc_100ns: hns(arrival),
        };
        flag = false;
        out.push(Pkt { leg, arrival, order: out.len(), info, samples: (!silent).then_some(samples), walls });
    }
    out
}

struct Run {
    out: Vec<[f32; 2]>,
    ep_walls: Vec<i64>,
    rf_index: HashMap<i64, i64>,
    status: Status,
    fault: Option<Fault>,
    max_buffered: u64,
    ran: bool,
}

fn run(sc: &Scenario) -> Run {
    let mut engine = Engine::new(sc.cfg.clone());
    let mut pk = packets(&sc.rf, Leg::Reference, sc.end, None, &|w, ch| (sc.own)(w, ch));
    pk.extend(packets(&sc.ep, Leg::Endpoint, sc.end, sc.slip_at, &|w, ch| sc.endpoint_sample(w, ch)));
    pk.sort_by_key(|p| (p.arrival, p.leg == Leg::Endpoint, p.order));

    let mut r = Run {
        out: vec![],
        ep_walls: vec![],
        rf_index: HashMap::new(),
        status: engine.status(),
        fault: None,
        max_buffered: 0,
        ran: false,
    };
    let mut rf_local = 0i64;
    for p in pk {
        match p.leg {
            Leg::Endpoint => r.ep_walls.extend(&p.walls),
            Leg::Reference => {
                for w in &p.walls {
                    r.rf_index.insert(*w, rf_local);
                    rf_local += 1;
                }
            }
        }
        let mut result = engine.push(p.leg, &p.info, p.samples.as_deref());
        while result.is_ok() {
            match engine.take_job() {
                Some(job) => result = engine.complete_job(run_align_job(&job)),
                None => break,
            }
        }
        while let Some(c) = engine.pop_chunk() {
            assert_eq!(c.len(), 960, "every chunk is 480 stereo frames");
            r.out.extend(c.chunks_exact(2).map(|f| [f[0], f[1]]));
        }
        let s = engine.status();
        if result.is_ok() {
            r.max_buffered = r.max_buffered.max(s.buffered_frames);
        }
        r.ran |= s.phase == Phase::Running;
        if let Err(f) = result {
            r.fault = Some(f);
            break;
        }
    }
    r.status = engine.status();
    assert!(r.max_buffered <= sc.cfg.max_pending_frames as u64);
    r
}

/// Every output frame is either muted or exactly the other-apps signal: own audio never leaks,
/// never gets injected inverted, and the endpoint is never passed raw while own audio is present.
/// Returns how many frames carried audio.
fn assert_no_leak(sc: &Scenario, r: &Run, before_wall: Option<i64>) -> usize {
    let mut audible = 0;
    for (k, o) in r.out.iter().enumerate() {
        let w = r.ep_walls[k];
        if before_wall.is_some_and(|b| w >= b) {
            break;
        }
        if *o == [0.0, 0.0] {
            continue;
        }
        audible += 1;
        for (ch, got) in o.iter().enumerate() {
            let want = (sc.other)(w, ch);
            assert!(
                (got - want).abs() <= 2e-6,
                "output frame {k} (wall {w}) ch {ch} = {got} but other apps = {want}"
            );
        }
    }
    audible
}

/// True offset for the final generation: the latest endpoint frame whose partner was captured.
fn expected_offset(sc: &Scenario, r: &Run) -> i64 {
    (0..r.ep_walls.len())
        .rev()
        .find_map(|i| r.rf_index.get(&(r.ep_walls[i] + sc.bias)).map(|j| j - i as i64))
        .expect("some endpoint frame has a captured reference partner")
}

fn assert_locked_exactly(sc: &Scenario, r: &Run) {
    assert!(r.fault.is_none(), "unexpected fault: {:?}", r.fault);
    assert_eq!(r.status.phase, Phase::Running, "{:#?}", r.status);
    assert_eq!(r.status.offset_frames, Some(expected_offset(sc, r)), "{:#?}", r.status);
}

#[test]
fn locks_the_hdmi_style_1024_frame_shift_and_leaves_only_other_apps() {
    // Packet timing says offset 7000; the content really sits 1024 frames further (not a multiple
    // of the 480-frame period), as on the measured HDMI endpoint.
    let sc = Scenario::new(1024);
    let r = run(&sc);
    assert_locked_exactly(&sc, &r);
    assert_eq!(expected_offset(&sc, &r), 7_000 + 1_024);
    let audible = assert_no_leak(&sc, &r, None);
    assert!(audible > 3 * F as usize, "only {audible} frames were subtracted");
    // The rechunker may still hold a partial chunk of subtracted frames.
    let held = r.status.subtracted_frames as usize - audible;
    assert!(held < 480, "{held} subtracted frames never reached a chunk");
}

#[test]
fn isolated_own_audio_cancels_to_bit_exact_zero() {
    let mut sc = Scenario::new(1024);
    sc.other = Box::new(|_, _| 0.0);
    let r = run(&sc);
    assert_locked_exactly(&sc, &r);
    assert!(r.status.subtracted_frames > 3 * F as u64);
    assert!(r.out.iter().all(|f| *f == [0.0, 0.0]), "unity subtraction of an identical copy is exactly zero");
}

#[test]
fn locks_arbitrary_integer_offsets_with_mismatched_packet_sizes() {
    for bias in [-868, 0, 1, 1023, 6720] {
        let mut sc = Scenario::new(bias);
        sc.ep = LegPlan::new(1_440, &[441, 480, 519], 300);
        sc.rf = LegPlan::new(0, &[480], 480);
        let r = run(&sc);
        assert_locked_exactly(&sc, &r);
        assert!(assert_no_leak(&sc, &r, None) > 2 * F as usize, "bias {bias}");
    }
}

#[test]
fn locks_with_read_time_only_timestamps() {
    let mut sc = Scenario::new(-480);
    sc.ep.read_time_only = true;
    sc.rf.read_time_only = true;
    let r = run(&sc);
    assert_locked_exactly(&sc, &r);
    assert_no_leak(&sc, &r, None);
}

#[test]
fn locks_under_a_louder_independent_other_source() {
    let mut sc = Scenario::new(1024);
    sc.other = Box::new(|w, ch| 2.0 * music(2, w, ch)); // +6 dB over own audio
    sc.end = 14 * F;
    let r = run(&sc);
    assert_locked_exactly(&sc, &r);
    assert!(assert_no_leak(&sc, &r, None) > F as usize);
}

#[test]
fn a_silent_reference_after_lock_passes_other_apps_bit_exactly() {
    let mut sc = Scenario::new(1024);
    sc.own = Box::new(|w, ch| if w < 3 * F { music(1, w, ch) } else { 0.0 });
    let r = run(&sc);
    assert_locked_exactly(&sc, &r);
    assert_no_leak(&sc, &r, None);
    let mut exact = 0;
    for (k, o) in r.out.iter().enumerate() {
        let w = r.ep_walls[k];
        if w >= 3 * F + 2 * 1024 && *o != [0.0, 0.0] {
            assert_eq!(*o, [(sc.other)(w, 0), (sc.other)(w, 1)]);
            exact += 1;
        }
    }
    assert!(exact > 2 * F as usize);
}

#[test]
fn never_passes_the_raw_endpoint_before_a_lock_even_when_own_audio_is_silent() {
    let mut sc = Scenario::new(1024);
    sc.own = Box::new(|_, _| 0.0);
    sc.end = 3 * F;
    let r = run(&sc);
    assert!(r.fault.is_none());
    assert_eq!(r.status.phase, Phase::Aligning);
    assert!(r.status.reason.contains("reference silent"), "{}", r.status.reason);
    assert!(!r.out.is_empty() && r.out.iter().all(|f| *f == [0.0, 0.0]));
    assert_eq!(r.status.passthrough_frames + r.status.subtracted_frames, 0);
}

#[test]
fn periodic_own_audio_is_ambiguous_and_never_locks() {
    let mut sc = Scenario::new(1024);
    sc.own = Box::new(|w, ch| music(1, w.rem_euclid(480), ch));
    sc.cfg.align_timeout_frames = 3 * F as u64;
    sc.end = 10 * F;
    let r = run(&sc);
    assert!(!r.ran && r.status.locks == 0);
    assert!(r.out.iter().all(|f| *f == [0.0, 0.0]));
    // Ambiguity is missing evidence, not a mismatch: stay muted, keep trying, say why.
    assert!(r.fault.is_none(), "{:?}", r.fault);
    assert_eq!(r.status.phase, Phase::Aligning);
    assert!(r.status.reason.contains("ambiguous"), "{}", r.status.reason);
}

#[test]
fn a_non_unity_path_is_rejected_not_fitted() {
    let mut sc = Scenario::new(1024);
    sc.gain = 0.8;
    sc.cfg.align_timeout_frames = 3 * F as u64;
    sc.end = 10 * F;
    let r = run(&sc);
    assert!(!r.ran && r.status.locks == 0);
    assert!(r.out.iter().all(|f| *f == [0.0, 0.0]));
    let f = r.fault.clone().expect("aligning must time out");
    assert!(f.0.contains("unity"), "{}", f.0);
}

#[test]
fn swapped_stereo_channels_never_lock() {
    let mut sc = Scenario::new(1024);
    sc.swap = true;
    sc.cfg.align_timeout_frames = 3 * F as u64;
    sc.end = 8 * F;
    let r = run(&sc);
    assert!(!r.ran && r.status.locks == 0);
    assert!(r.out.iter().all(|f| *f == [0.0, 0.0]));
}

#[test]
fn much_louder_other_audio_never_mislocks_or_faults() {
    let mut sc = Scenario::new(1024);
    sc.other = Box::new(|w, ch| 4.0 * music(2, w, ch)); // +12 dB over own audio
    sc.end = 8 * F;
    let r = run(&sc);
    assert!(r.fault.is_none(), "{:?}", r.fault);
    if r.status.phase == Phase::Running {
        assert_eq!(r.status.offset_frames, Some(expected_offset(&sc, &r)));
    }
    assert_no_leak(&sc, &r, None);
}

#[test]
fn an_unflagged_reference_gap_ends_the_generation_and_is_not_treated_as_silence() {
    let mut sc = Scenario::new(1024);
    sc.rf.drops = vec![(3 * F, 3 * F + F / 5)];
    sc.end = 8 * F;
    let r = run(&sc);
    assert_locked_exactly(&sc, &r);
    assert!(r.status.generation >= 1 && r.status.timeline_gaps >= 1);
    assert!(r.status.locks >= 2, "relocked after the gap");
    assert_no_leak(&sc, &r, None);
}

#[test]
fn a_flagged_endpoint_discontinuity_realigns() {
    let mut sc = Scenario::new(-868);
    sc.ep.drops = vec![(3 * F, 3 * F + 2_400)];
    sc.ep.flag_drops = true;
    sc.end = 8 * F;
    let r = run(&sc);
    assert_locked_exactly(&sc, &r);
    assert!(r.status.discontinuities >= 1 && r.status.locks >= 2, "{:#?}", r.status);
    assert_no_leak(&sc, &r, None);
}

#[test]
fn a_stalled_reference_overflows_into_a_fault() {
    let mut sc = Scenario::new(1024);
    sc.rf.stop_at = Some(4 * F);
    let r = run(&sc);
    assert!(r.ran);
    let f = r.fault.clone().expect("reference stall must fault");
    assert!(f.0.contains("overflow"), "{}", f.0);
    assert_eq!(r.status.phase, Phase::Failed);
    assert_no_leak(&sc, &r, None);
}

#[test]
fn a_reference_leg_that_never_delivers_faults() {
    let mut sc = Scenario::new(0);
    sc.rf.stop_at = Some(0);
    sc.end = 4 * F;
    let r = run(&sc);
    let f = r.fault.clone().expect("no reference must fault");
    assert!(f.0.contains("no packets"), "{}", f.0);
    assert!(r.out.iter().all(|f| *f == [0.0, 0.0]));
}

#[test]
fn an_undetected_one_frame_slip_is_caught_as_lost_lock() {
    let mut sc = Scenario::new(1024);
    sc.slip_at = Some(4 * F);
    sc.end = 10 * F;
    let r = run(&sc);
    assert!(r.ran);
    assert_no_leak(&sc, &r, Some(4 * F));
    let f = r.fault.clone().expect("a slip must be detected");
    assert!(f.0.contains("lost lock"), "{}", f.0);
}

#[test]
fn a_proven_bias_lets_a_silent_reference_pass_after_an_endpoint_idle_gap() {
    let mut sc = Scenario::new(1024);
    // Own audio for 2.5 s, silence, then own audio again from 6 s.
    sc.own = Box::new(|w, ch| if (5 * F / 2..6 * F).contains(&w) { 0.0 } else { music(1, w, ch) });
    // Endpoint idle (no packets, no flag) for a second.
    sc.ep.drops = vec![(3 * F + F / 2, 4 * F + F / 2)];
    sc.end = 10 * F;
    let r = run(&sc);
    assert_locked_exactly(&sc, &r);
    assert!(r.status.generation >= 1);
    assert!(r.status.passthrough_frames > F as u64 / 2, "passthrough {}", r.status.passthrough_frames);
    assert!(r.status.locks >= 2, "relocked when own audio came back");
    assert_no_leak(&sc, &r, None);
}
