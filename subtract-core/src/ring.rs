//! One leg's recent frames: a bounded, *contiguous* window `[start, end)` of that leg's local frame
//! indices. Contiguity is the invariant the whole engine leans on — a timeline gap ends the
//! generation and resets the ring, so any frame inside the window is real captured data (a
//! `SILENT` packet is real data too, stored as zeros). Nothing is ever zero-filled for a frame the
//! engine did not hand us.

pub(crate) struct FrameRing {
    cap: usize,
    samples: Vec<f32>,
    /// Per slot: running count of non-zero frames strictly before that frame. Differences of these
    /// answer "is this range all exact zeros?" in O(1).
    nonzero_before: Vec<u64>,
    start: u64,
    end: u64,
    nonzero_total: u64,
}

impl FrameRing {
    pub(crate) fn new(cap: usize) -> Self {
        assert!(cap > 0);
        FrameRing {
            cap,
            samples: vec![0.0; cap * 2],
            nonzero_before: vec![0; cap],
            start: 0,
            end: 0,
            nonzero_total: 0,
        }
    }

    pub(crate) fn start(&self) -> u64 {
        self.start
    }

    pub(crate) fn end(&self) -> u64 {
        self.end
    }

    /// Forget everything; the next pushed frame has index `at`.
    pub(crate) fn reset(&mut self, at: u64) {
        self.start = at;
        self.end = at;
    }

    fn slot(&self, idx: u64) -> usize {
        (idx % self.cap as u64) as usize
    }

    /// Append `frames` interleaved stereo frames (`None` = a silent packet). Rejects the whole
    /// packet, appending nothing, if any sample is non-finite. The oldest frames fall off the front
    /// once `cap` is reached — the ring never grows.
    pub(crate) fn push(&mut self, data: Option<&[f32]>, frames: usize) -> Result<(), ()> {
        if let Some(d) = data {
            if d.len() != frames * 2 || d.iter().any(|v| !v.is_finite()) {
                return Err(());
            }
        }
        for f in 0..frames {
            let (l, r) = match data {
                Some(d) => (d[2 * f], d[2 * f + 1]),
                None => (0.0, 0.0),
            };
            let s = self.slot(self.end);
            self.samples[2 * s] = l;
            self.samples[2 * s + 1] = r;
            self.nonzero_before[s] = self.nonzero_total;
            if l != 0.0 || r != 0.0 {
                self.nonzero_total += 1;
            }
            self.end += 1;
            if self.end - self.start > self.cap as u64 {
                self.start = self.end - self.cap as u64;
            }
        }
        Ok(())
    }

    /// Whether `[a, b)` (signed, so callers can pass offset-mapped indices) lies inside the window.
    pub(crate) fn contains(&self, a: i64, b: i64) -> bool {
        a <= b && a >= self.start as i64 && b <= self.end as i64
    }

    pub(crate) fn frame(&self, idx: u64) -> [f32; 2] {
        debug_assert!(idx >= self.start && idx < self.end, "frame {idx} outside ring");
        let s = self.slot(idx);
        [self.samples[2 * s], self.samples[2 * s + 1]]
    }

    fn count_before(&self, idx: u64) -> u64 {
        if idx == self.end {
            self.nonzero_total
        } else {
            self.nonzero_before[self.slot(idx)]
        }
    }

    /// Number of frames in `[a, b)` with any non-zero sample. The range must be inside the window.
    pub(crate) fn nonzero_in(&self, a: u64, b: u64) -> u64 {
        debug_assert!(self.contains(a as i64, b as i64));
        self.count_before(b) - self.count_before(a)
    }

    pub(crate) fn copy_interleaved(&self, a: u64, b: u64, out: &mut Vec<f32>) {
        debug_assert!(self.contains(a as i64, b as i64));
        out.reserve(((b - a) * 2) as usize);
        for idx in a..b {
            out.extend_from_slice(&self.frame(idx));
        }
    }

    /// Per-channel sum of squares over `[a, b)`.
    pub(crate) fn energy(&self, a: u64, b: u64) -> [f64; 2] {
        debug_assert!(self.contains(a as i64, b as i64));
        let mut e = [0.0f64; 2];
        for idx in a..b {
            let f = self.frame(idx);
            e[0] += f64::from(f[0]) * f64::from(f[0]);
            e[1] += f64::from(f[1]) * f64::from(f[1]);
        }
        e
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stays_bounded_and_counts_nonzero_frames_across_wraparound() {
        let mut r = FrameRing::new(8);
        r.push(Some(&[1.0, 0.0, 0.0, 0.0, 0.0, -2.0]), 3).unwrap(); // frames 0..3: nz, z, nz
        r.push(None, 4).unwrap(); // frames 3..7 silent
        r.push(Some(&[0.5, 0.5, 0.0, 0.0, 0.0, 0.25]), 3).unwrap(); // frames 7..10: nz, z, nz
        assert_eq!((r.start(), r.end()), (2, 10));
        assert_eq!(r.nonzero_in(2, 10), 3); // frame 2, 7, 9
        assert_eq!(r.nonzero_in(3, 7), 0);
        assert_eq!(r.nonzero_in(8, 9), 0);
        assert_eq!(r.frame(9), [0.0, 0.25]);
        assert!(!r.contains(1, 4));
        assert!(r.contains(2, 10));
    }

    #[test]
    fn rejects_non_finite_packets_without_appending() {
        let mut r = FrameRing::new(4);
        assert!(r.push(Some(&[0.0, f32::NAN]), 1).is_err());
        assert!(r.push(Some(&[0.0, 1.0, 2.0]), 1).is_err());
        assert_eq!(r.end(), 0);
    }

    #[test]
    fn reset_empties_the_window_but_keeps_counting_indices() {
        let mut r = FrameRing::new(4);
        r.push(Some(&[1.0, 1.0, 1.0, 1.0]), 2).unwrap();
        r.reset(r.end());
        assert_eq!((r.start(), r.end()), (2, 2));
        r.push(None, 1).unwrap();
        assert_eq!(r.nonzero_in(2, 3), 0);
    }
}
