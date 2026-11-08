//! Fractional-octave smoothing.
//!
//! Weighting by a 6th-order Butterworth band-pass around each point, slopes truncated at −20 dB:
//!
//! * band edges `fc·2^(∓1/(2N))`, weight `w = |H(f)|` of the band-pass
//!   (3rd order per side: `|H|² = 1 / (1 + q⁶)`, `q = (f² − fc²) / (f·B)`),
//!   points with `|H|² < 0.01` (−20 dB) excluded;
//! * magnitude = `Σ w·|x| / Σ w` (average of linear magnitude);
//! * phase = `arg(Σ w·x)` (argument of the weighted complex average).

use crate::response::Response;
use num_complex::Complex64;

/// Smoothing choices.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Smoothing {
    #[default]
    None,
    /// 1/N octave.
    Oct(u32),
}

pub fn smooth(r: &Response, s: Smoothing) -> Response {
    let Smoothing::Oct(n) = s else { return r.clone() };
    let half = 0.5 / n as f64;
    let freq = &r.freq;
    let mut out = Vec::with_capacity(r.len());
    for &fc in freq {
        let f1 = fc * 2f64.powf(-half);
        let f2 = fc * 2f64.powf(half);
        let bw = f2 - f1;
        let (mut wsum, mut msum, mut csum) = (0.0, 0.0, Complex64::new(0.0, 0.0));
        // |H|² ≥ 0.01 ⇔ q⁶ ≤ 99; bound the scan with a generous log window.
        let lo = freq.partition_point(|&f| f < fc / 4.0);
        let hi = freq.partition_point(|&f| f <= fc * 4.0);
        for (&f, &x) in freq[lo..hi].iter().zip(&r.values[lo..hi]) {
            let q = (f * f - fc * fc) / (f * bw);
            let h2 = 1.0 / (1.0 + q.powi(6));
            if h2 < 0.01 {
                continue;
            }
            let w = h2.sqrt();
            wsum += w;
            msum += w * x.norm();
            csum += x * w;
        }
        out.push(Complex64::from_polar(msum / wsum, csum.arg()));
    }
    r.with_values(r.name.clone(), out)
}
