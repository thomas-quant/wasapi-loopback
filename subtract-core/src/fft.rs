//! Minimal iterative radix-2 complex FFT (f64). Only used for the bounded correlation search, so
//! it favours obvious correctness over speed; the twiddle table is computed directly per call
//! rather than by recurrence so precision does not degrade with size.

pub(crate) fn fft_in_place(re: &mut [f64], im: &mut [f64], inverse: bool) {
    let n = re.len();
    assert_eq!(n, im.len());
    assert!(n.is_power_of_two(), "fft size must be a power of two");
    if n < 2 {
        return;
    }

    // Bit-reversal permutation.
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }

    let sign = if inverse { 1.0 } else { -1.0 };
    let half = n / 2;
    let (tw_re, tw_im): (Vec<f64>, Vec<f64>) = (0..half)
        .map(|k| {
            let a = 2.0 * std::f64::consts::PI * k as f64 / n as f64;
            (a.cos(), sign * a.sin())
        })
        .unzip();

    let mut len = 2;
    while len <= n {
        let step = n / len;
        let h = len / 2;
        for start in (0..n).step_by(len) {
            for k in 0..h {
                let (wr, wi) = (tw_re[k * step], tw_im[k * step]);
                let a = start + k;
                let b = a + h;
                let xr = re[b] * wr - im[b] * wi;
                let xi = re[b] * wi + im[b] * wr;
                re[b] = re[a] - xr;
                im[b] = im[a] - xi;
                re[a] += xr;
                im[a] += xi;
            }
        }
        len <<= 1;
    }

    if inverse {
        let scale = 1.0 / n as f64;
        for v in re.iter_mut().chain(im.iter_mut()) {
            *v *= scale;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_a_direct_dft_and_round_trips() {
        let n = 64;
        let x: Vec<f64> = (0..n).map(|i| ((i * 37 % 11) as f64 - 5.0) / 3.0).collect();
        let (mut re, mut im) = (x.clone(), vec![0.0; n]);
        fft_in_place(&mut re, &mut im, false);
        for k in 0..n {
            let (mut dr, mut di) = (0.0, 0.0);
            for (t, v) in x.iter().enumerate() {
                let a = -2.0 * std::f64::consts::PI * (k * t) as f64 / n as f64;
                dr += v * a.cos();
                di += v * a.sin();
            }
            assert!((re[k] - dr).abs() < 1e-9 && (im[k] - di).abs() < 1e-9, "bin {k}");
        }
        fft_in_place(&mut re, &mut im, true);
        for i in 0..n {
            assert!((re[i] - x[i]).abs() < 1e-12 && im[i].abs() < 1e-12);
        }
    }
}
