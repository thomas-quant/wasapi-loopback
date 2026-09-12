//! Bounded integer-offset search by normalized cross-correlation of naturally occurring own audio.
//!
//! A job is one endpoint calibration block `C[i0 .. i0+N)` and the reference segment covering every
//! offset in `[base_offset, base_offset + 2·radius]`. The search is a pure function of that data, so
//! the native owner runs it on a worker thread while both legs keep flowing.
//!
//! The job only nominates a candidate. It is conservative about *which* peak (boundary, weak and
//! ambiguous peaks are rejected) and says nothing about gain or exactness: the engine then checks
//! the candidate on disjoint, later stereo data at unity gain before any sample is subtracted.

use crate::fft::fft_in_place;

#[derive(Clone, Copy, Debug)]
pub struct AlignParams {
    /// Reject when the best competing local maximum (outside `lobe_guard`) reaches this fraction of
    /// the main peak — periodic content cannot pin an integer offset.
    pub ambiguity_ratio: f64,
    /// Below this normalized correlation the peak is not taken as evidence at all.
    pub min_peak_correlation: f64,
    /// Half-width (frames) of the main lobe excluded from the competitor search.
    pub lobe_guard: usize,
    /// A peak this close to either end of the search window may really lie outside it.
    pub edge_guard: usize,
}

#[derive(Clone, Debug)]
pub struct AlignJob {
    pub id: u64,
    pub generation: u32,
    /// Endpoint local index of `endpoint[0]`.
    pub endpoint_start: u64,
    /// Offset (reference index − endpoint index) that correlation lag 0 corresponds to.
    pub base_offset: i64,
    /// Interleaved stereo, N frames.
    pub endpoint: Vec<f32>,
    /// Interleaved stereo, N + 2·radius frames, starting at reference index
    /// `endpoint_start + base_offset`.
    pub reference: Vec<f32>,
    pub params: AlignParams,
}

#[derive(Clone, Debug, PartialEq)]
pub enum AlignOutcome {
    /// `offset` = reference index − endpoint index of the best integer alignment.
    Candidate { offset: i64, peak: f64, runner_up: f64 },
    /// `mismatch` marks evidence that the endpoint does not carry the reference (as opposed to
    /// merely not enough, or ambiguous, evidence). Only mismatches can end a session on timeout.
    Rejected { reason: String, mismatch: bool },
}

#[derive(Clone, Debug)]
pub struct AlignResult {
    pub id: u64,
    pub generation: u32,
    pub outcome: AlignOutcome,
}

fn channel(interleaved: &[f32], ch: usize) -> Vec<f64> {
    interleaved.iter().skip(ch).step_by(2).map(|v| f64::from(*v)).collect()
}

/// `out[k] = Σ_n x[n]·y[n+k]` for `k < lags`; needs `y.len() >= x.len() + lags - 1`.
pub(crate) fn xcorr(x: &[f64], y: &[f64], lags: usize) -> Vec<f64> {
    assert!(y.len() + 1 >= x.len() + lags);
    // Circular correlation of zero-padded inputs equals the linear one for these lags as long as
    // the transform covers y: n + k <= x.len() - 1 + lags - 1 <= y.len() - 1 < size.
    let size = y.len().next_power_of_two().max(2);
    let (mut xr, mut xi) = (vec![0.0; size], vec![0.0; size]);
    let (mut yr, mut yi) = (vec![0.0; size], vec![0.0; size]);
    xr[..x.len()].copy_from_slice(x);
    yr[..y.len()].copy_from_slice(y);
    fft_in_place(&mut xr, &mut xi, false);
    fft_in_place(&mut yr, &mut yi, false);
    // conj(X)·Y
    for k in 0..size {
        let (a, b, c, d) = (xr[k], -xi[k], yr[k], yi[k]);
        yr[k] = a * c - b * d;
        yi[k] = a * d + b * c;
    }
    fft_in_place(&mut yr, &mut yi, true);
    yr.truncate(lags);
    yr
}

pub fn run_align_job(job: &AlignJob) -> AlignResult {
    AlignResult {
        id: job.id,
        generation: job.generation,
        outcome: search(job),
    }
}

fn search(job: &AlignJob) -> AlignOutcome {
    let n = job.endpoint.len() / 2;
    let m = job.reference.len() / 2;
    if n == 0 || m < n {
        return AlignOutcome::Rejected { reason: "empty alignment job".into(), mismatch: false };
    }
    let lags = m - n + 1;
    let p = job.params;

    let mut num = vec![0.0f64; lags];
    let mut ey = vec![0.0f64; lags];
    let mut ex = 0.0f64;
    for ch in 0..2 {
        let x = channel(&job.endpoint, ch);
        let y = channel(&job.reference, ch);
        ex += x.iter().map(|v| v * v).sum::<f64>();
        for (acc, v) in num.iter_mut().zip(xcorr(&x, &y, lags)) {
            *acc += v;
        }
        let mut prefix = Vec::with_capacity(m + 1);
        prefix.push(0.0f64);
        for v in &y {
            prefix.push(prefix.last().unwrap() + v * v);
        }
        for (k, acc) in ey.iter_mut().enumerate() {
            *acc += prefix[k + n] - prefix[k];
        }
    }
    if ex <= 0.0 {
        return AlignOutcome::Rejected {
            reason: "endpoint block is digital silence while the reference is active".into(),
            mismatch: true,
        };
    }

    let rho: Vec<f64> = num
        .iter()
        .zip(&ey)
        .map(|(c, e)| if *e > 0.0 { c / (ex * e).sqrt() } else { 0.0 })
        .collect();
    let (best, peak) = rho
        .iter()
        .copied()
        .enumerate()
        .fold((0usize, f64::NEG_INFINITY), |acc, (k, v)| if v > acc.1 { (k, v) } else { acc });

    if !peak.is_finite() || peak < p.min_peak_correlation {
        // Could be own audio drowned by other apps as easily as own audio missing: not a mismatch.
        return AlignOutcome::Rejected {
            reason: format!(
                "no correlation peak (best {peak:.3} < {:.3}); own audio not found on the endpoint",
                p.min_peak_correlation
            ),
            mismatch: false,
        };
    }
    if best < p.edge_guard || best + p.edge_guard >= lags {
        return AlignOutcome::Rejected {
            reason: format!(
                "correlation peak at the search boundary (offset {}); the true offset may lie outside the window",
                job.base_offset + best as i64
            ),
            mismatch: true,
        };
    }

    // Best competing local maximum outside the main lobe.
    let mut runner_up = f64::NEG_INFINITY;
    let mut runner_at = best;
    for k in 0..lags {
        if k.abs_diff(best) <= p.lobe_guard {
            continue;
        }
        let left = if k > 0 { rho[k - 1] } else { f64::NEG_INFINITY };
        let right = if k + 1 < lags { rho[k + 1] } else { f64::NEG_INFINITY };
        if rho[k] >= left && rho[k] >= right && rho[k] > runner_up {
            runner_up = rho[k];
            runner_at = k;
        }
    }
    if runner_up > p.ambiguity_ratio * peak {
        return AlignOutcome::Rejected {
            reason: format!(
                "ambiguous correlation: peak {peak:.3} at offset {} but {runner_up:.3} at offset {} \
                 (periodic own audio cannot pin an integer offset)",
                job.base_offset + best as i64,
                job.base_offset + runner_at as i64
            ),
            mismatch: false,
        };
    }

    AlignOutcome::Candidate {
        offset: job.base_offset + best as i64,
        peak,
        runner_up: runner_up.max(0.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> AlignParams {
        AlignParams {
            ambiguity_ratio: 0.9,
            min_peak_correlation: 0.05,
            lobe_guard: 32,
            edge_guard: 64,
        }
    }

    fn noise(seed: u64, i: u64) -> f32 {
        let mut z = seed.wrapping_add(i.wrapping_mul(0x9E37_79B9_7F4A_7C15));
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        (z >> 40) as f32 / (1u64 << 23) as f32 - 1.0
    }

    #[test]
    fn fft_correlation_matches_direct_sum() {
        let x: Vec<f64> = (0..300).map(|i| f64::from(noise(1, i))).collect();
        let y: Vec<f64> = (0..420).map(|i| f64::from(noise(2, i))).collect();
        let lags = 121;
        let fast = xcorr(&x, &y, lags);
        for k in 0..lags {
            let direct: f64 = (0..x.len()).map(|n| x[n] * y[n + k]).sum();
            assert!((fast[k] - direct).abs() < 1e-9, "lag {k}: {} vs {direct}", fast[k]);
        }
    }

    fn job(offset_true: i64, n: usize, radius: usize, own: impl Fn(u64, usize) -> f32) -> AlignJob {
        // Reference index j = endpoint index i + offset_true.
        let base_offset = offset_true - radius as i64 + 7; // truth is not centred
        let i0 = 50_000u64;
        let r0 = (i0 as i64 + base_offset) as u64;
        let mut endpoint = Vec::new();
        for i in 0..n as u64 {
            let j = (i0 + i) as i64 + offset_true;
            for ch in 0..2 {
                endpoint.push(own(j as u64, ch) + 0.5 * noise(99 + ch as u64, i0 + i));
            }
        }
        let mut reference = Vec::new();
        for j in 0..(n + 2 * radius) as u64 {
            for ch in 0..2 {
                reference.push(own(r0 + j, ch));
            }
        }
        AlignJob { id: 1, generation: 0, endpoint_start: i0, base_offset, endpoint, reference, params: params() }
    }

    #[test]
    fn finds_the_exact_offset_with_an_independent_other_source() {
        let own = |j: u64, ch: usize| 0.5 * noise(7 + ch as u64, j);
        for truth in [1024i64, -868, 6720, 1] {
            match run_align_job(&job(truth, 8192, 2048, own)).outcome {
                AlignOutcome::Candidate { offset, .. } => assert_eq!(offset, truth),
                other => panic!("{truth}: {other:?}"),
            }
        }
    }

    #[test]
    fn periodic_reference_is_rejected_as_ambiguous() {
        let own = |j: u64, ch: usize| 0.5 * noise(7 + ch as u64, j % 480);
        match run_align_job(&job(1024, 8192, 2048, own)).outcome {
            AlignOutcome::Rejected { reason, mismatch } => {
                assert!(reason.contains("ambiguous") && !mismatch, "{reason}")
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn absent_own_audio_is_rejected_not_guessed() {
        let mut j = job(0, 4096, 1024, |j, ch| 0.5 * noise(7 + ch as u64, j));
        // Endpoint carries only unrelated audio.
        for (k, v) in j.endpoint.iter_mut().enumerate() {
            *v = noise(1234, k as u64);
        }
        assert!(matches!(run_align_job(&j).outcome, AlignOutcome::Rejected { .. }));
    }
}
