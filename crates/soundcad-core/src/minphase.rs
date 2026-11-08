//! Minimum phase from magnitude (Bode gain–phase relation), computed on the log frequency grid.
//!
//! φ(u₀) = (1/π) ∫ dα/du · ln coth(|u − u₀|/2) du, with u = ln ω and α = ln|H| (nepers): midpoint rule
//! between grid points (slope of each segment × kernel at the segment midpoint × segment width).
//! Outside the data the magnitude continues with constant slopes s (dB/oct) estimated at both ends;
//! a constant slope n = s/6.02 (power of ω) over u < a adds (π/2)·n·K(a − u₀), over u > b adds
//! (π/2)·n·(1 − K(b − u₀)), where K(x) = (2/π²)∫_{−∞}^{x} ln coth(|u|/2) du is the normalized kernel
//! integral (closed form with Legendre's χ₂: K(x) = (4/π²)·χ₂(e^{−|x|}) for x ≤ 0, 1 − that for x > 0).
//! Slopes: least-squares line of dB vs grid index over the lowest and the highest octave × points per octave.

use std::f64::consts::PI;

/// Legendre chi: χ₂(z) = Σ z^(2k+1)/(2k+1)², 0 ≤ z ≤ 1.
fn chi2(z: f64) -> f64 {
    if z >= 1.0 {
        return PI * PI / 8.0;
    }
    let (z2, mut p, mut s) = (z * z, z, 0.0);
    let mut k = 1.0;
    while p > 1e-17 {
        s += p / (k * k);
        p *= z2;
        k += 2.0;
    }
    s
}

/// Normalized integral of the Bode kernel from −∞ to x (0 … 1, ½ at 0).
fn kernel_integral(x: f64) -> f64 {
    let v = 4.0 / (PI * PI) * chi2((-x.abs()).exp());
    if x <= 0.0 { v } else { 1.0 - v }
}

/// Slope of the least-squares line through `points` (x, y); 0 for fewer than two distinct x.
pub fn fit_slope(points: impl Iterator<Item = (f64, f64)>) -> f64 {
    let (mut n, mut sx, mut sy, mut sxx, mut sxy) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for (x, y) in points {
        n += 1.0;
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
    }
    let d = n * sxx - sx * sx;
    if d.abs() < 1e-12 { 0.0 } else { (n * sxy - sx * sy) / d }
}

/// Minimum phase [rad, unwrapped] of linear magnitude `mag` at `freq` (ascending, log-spaced),
/// with the given end slopes [dB/oct] for the magnitude outside the data.
pub fn minimum_phase(freq: &[f64], mag: &[f64], low_slope: f64, high_slope: f64) -> Vec<f64> {
    let n = freq.len();
    let u: Vec<f64> = freq.iter().map(|f| (2.0 * PI * f).ln()).collect();
    let a: Vec<f64> = mag.iter().map(|m| m.max(1e-15).ln()).collect();
    let (nl, nh) = (low_slope / 6.02, high_slope / 6.02);
    let weight = |mid_minus_uj: f64| {
        let x = mid_minus_uj.abs().exp();
        (2.0 / (PI * PI)) * ((x + 1.0) / (x - 1.0)).ln()
    };
    // log-uniform grid: the weight depends only on k − j, tabulate it once (index k − j + n)
    let h = (u[n - 1] - u[0]) / (n - 1) as f64;
    let uniform = n > 2 && u.windows(2).all(|w| ((w[1] - w[0]) - h).abs() < 1e-9 * h.abs().max(1.0));
    let table: Vec<f64> = if uniform { (0..2 * n).map(|m| weight((m as f64 - n as f64 - 0.5) * h)).collect() } else { vec![] };
    (0..n)
        .map(|j| {
            let mut s = 0.0;
            for k in 1..n {
                // (slope of the segment) × kernel at its midpoint × width = Δα × kernel
                let w = if uniform { table[k + n - j] } else { weight(0.5 * (u[k] + u[k - 1]) - u[j]) };
                s += (a[k] - a[k - 1]) * w;
            }
            PI / 2.0 * (s + nl * kernel_integral(u[0] - u[j]) + nh * (1.0 - kernel_integral(u[n - 1] - u[j])))
        })
        .collect()
}
