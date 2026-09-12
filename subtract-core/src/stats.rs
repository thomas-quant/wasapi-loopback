//! Equivalence-style decisions over per-sub-block estimates.
//!
//! Each sub-block yields one estimate (a unity-gain ratio, or a fractional-delay projection). The
//! other apps in the endpoint mix are independent of the reference, so they only add noise to these
//! estimates; the sub-block dispersion measures that noise. A verdict is:
//!
//! * `Accept`  — the whole `mean ± z·se` interval lies inside `target ± tol`;
//! * `Reject`  — the interval lies wholly outside it;
//! * `Inconclusive` — otherwise. The engine then waits for more data; it never "fits" its way in.
//!
//! Sub-blocks of real audio are not independent, so `se` is optimistic; the tolerances and `z`
//! values in `Config` are chosen with that in mind, and nothing downstream treats `se` as a
//! calibrated probability.

use std::collections::VecDeque;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Verdict {
    Accept,
    Reject,
    Inconclusive,
}

#[derive(Clone, Debug)]
pub(crate) struct Evidence {
    vals: VecDeque<f64>,
    cap: usize,
}

impl Evidence {
    pub(crate) fn new(cap: usize) -> Self {
        Evidence {
            vals: VecDeque::with_capacity(cap.max(1)),
            cap: cap.max(1),
        }
    }

    pub(crate) fn push(&mut self, v: f64) {
        if !v.is_finite() {
            return;
        }
        if self.vals.len() == self.cap {
            self.vals.pop_front();
        }
        self.vals.push_back(v);
    }

    pub(crate) fn clear(&mut self) {
        self.vals.clear();
    }

    pub(crate) fn len(&self) -> usize {
        self.vals.len()
    }

    /// `(mean, standard error)` — `None` below two values.
    pub(crate) fn summary(&self) -> Option<(f64, f64)> {
        let n = self.vals.len();
        if n < 2 {
            return None;
        }
        let mean = self.vals.iter().sum::<f64>() / n as f64;
        let var = self.vals.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / (n - 1) as f64;
        Some((mean, (var / n as f64).sqrt()))
    }

    pub(crate) fn mean(&self) -> Option<f64> {
        (!self.vals.is_empty()).then(|| self.vals.iter().sum::<f64>() / self.vals.len() as f64)
    }

    pub(crate) fn verdict(&self, target: f64, tol: f64, z: f64, min_n: usize) -> Verdict {
        if self.vals.len() < min_n.max(2) {
            return Verdict::Inconclusive;
        }
        let Some((mean, se)) = self.summary() else {
            return Verdict::Inconclusive;
        };
        let dev = (mean - target).abs();
        if dev + z * se <= tol {
            Verdict::Accept
        } else if dev - z * se > tol {
            Verdict::Reject
        } else {
            Verdict::Inconclusive
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn filled(vals: &[f64]) -> Evidence {
        let mut e = Evidence::new(64);
        vals.iter().for_each(|v| e.push(*v));
        e
    }

    #[test]
    fn exact_unity_accepts_and_a_clear_mismatch_rejects() {
        assert_eq!(filled(&[1.0; 8]).verdict(1.0, 0.02, 2.0, 8), Verdict::Accept);
        assert_eq!(filled(&[0.8, 0.81, 0.79, 0.8, 0.8, 0.8, 0.8, 0.8]).verdict(1.0, 0.02, 2.0, 8), Verdict::Reject);
    }

    #[test]
    fn noisy_or_short_evidence_is_inconclusive_not_accepted() {
        assert_eq!(filled(&[1.0; 7]).verdict(1.0, 0.02, 2.0, 8), Verdict::Inconclusive);
        let noisy = [1.0, 1.1, 0.9, 1.05, 0.95, 1.08, 0.92, 1.0];
        assert_eq!(filled(&noisy).verdict(1.0, 0.02, 2.0, 8), Verdict::Inconclusive);
    }

    #[test]
    fn window_is_bounded_and_ignores_non_finite_values() {
        let mut e = Evidence::new(3);
        for v in [5.0, f64::NAN, 1.0, 1.0, 1.0] {
            e.push(v);
        }
        assert_eq!(e.len(), 3);
        assert_eq!(e.mean(), Some(1.0));
    }
}
