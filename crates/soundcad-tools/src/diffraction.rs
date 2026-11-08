//! Diffraction: cabinet impact response of a baffle.
//!
//! * 72 rays from the driver at 0°, 5°, … hit the baffle outline at distance `d`;
//!   each edge point re-radiates to the listener over `l`. Listener = `axis_distance` r from
//!   the baffle point `mic`, direction (−sin h·cos v, −sin v, cos h·cos v) for
//!   `angle_hor` h, `angle_ver` v.
//! * Response relative to free field at r, delay r removed:
//!   `(r/rd)·e^{−jk(rd−r)}·[(1+cosθ)·P(ka·sinθ) − cosθ·(1/72)·Σ (rd/L)·P(ka)·E·e^{−jk(L−rd)}]`
//!   with `L = d + l − R/2`, rd the direct distance, θ the listener angle off the driver axis,
//!   `P` = piston directivity 2·J1(ka·sinθ)/(ka·sinθ) (a = Dd/2), real with sign; c = 344 m/s. At θ = 90°
//!   only the piston remains.
//! * θ composed as a = h + atan2(δx, d − δz), b = v + atan2(δy, d − δz) (δ = driver
//!   offset from the mic in the listener frame), cos θ = cos a·cos b.
//! * Rounded edge radius R: `E = 1/√(1 + (f/f0)^2.5)`, `f0 = c/(8R)`; ideal edge
//!   `E = 1, R = 0`.
//! * A ray aimed exactly at a corner (e.g. driver centred on a square) keeps the corner as its edge point.
//!
//! * Angular weighting: direct wave `1 + cos θ`, edge sources `cos θ` (subtracted while θ ≤ 90°, added beyond).
//! * Listener behind the baffle (θ > 90°): the direct sound travels the shortest edge path (Keller's
//!   geometrical theory of diffraction).
//! * Piston: Rayleigh directivity 2·J1(ka·sinθ)/(ka·sinθ), real with sign, a = Dd/2; the edge sources are
//!   at 90° off the axis.
//!
//! * Several drivers: mean of the per-driver responses.
//! * Open baffle: rear wave doubles the ray sum (`2·Σ`).
//!
//! * Reflections: floor / side wall: whole baffle turned toward the listener's mirror image (angle
//!   acos(R/d), distance d = √(R² + (2·offset)²)), each × 10^(−absorption/20).
//! * Full space: half-space response × cabinet impact / 2.
//!
//! * Directivity export: [`Baffle::directivity`] (angle sign flipped vs baffle rotation).

use num_complex::Complex64;
use std::f64::consts::PI;
use soundcad_core::response::db_to_lin;
use soundcad_core::{Plane, Response, C_AIR};

/// Baffle: outline, drivers, listening point and options; lengths in metres.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Baffle {
    pub corners: Vec<(f64, f64)>,
    pub drivers: Vec<(f64, f64)>,
    pub mic: (f64, f64),
    /// Driver effective diameter Dd.
    pub dd: f64,
    /// Edge radius (0 with an ideal edge).
    pub edge_radius: f64,
    pub axis_distance: f64,
    /// Axis angles (radians).
    pub angle_hor: f64,
    pub angle_ver: f64,
    /// Thin open baffle (dipole).
    pub open: bool,
    /// Reflecting floor at y and side wall at x (reflections).
    pub floor_y: Option<f64>,
    pub wall_x: Option<f64>,
    /// Reflection loss [dB].
    pub absorption_db: f64,
}

impl Baffle {
    /// Where the 72 rays from a driver leave the baffle.
    fn edge_points(&self, (dx, dy): (f64, f64)) -> Vec<(f64, f64)> {
        let n = self.corners.len();
        (0..72)
            .filter_map(|i| {
                let (uy, ux) = (i as f64 * 5.0).to_radians().sin_cos();
                let mut best = f64::INFINITY;
                for k in 0..n {
                    let (p, q) = (self.corners[k], self.corners[(k + 1) % n]);
                    let (ex, ey, wx, wy) = (q.0 - p.0, q.1 - p.1, p.0 - dx, p.1 - dy);
                    let den = ux * ey - uy * ex;
                    if den.abs() < 1e-15 {
                        continue;
                    }
                    let t = (wx * ey - wy * ex) / den;
                    let s = (wx * uy - wy * ux) / den;
                    if t > 1e-12 && (-1e-12..=1.0 + 1e-12).contains(&s) {
                        best = best.min(t);
                    }
                }
                best.is_finite().then(|| (dx + best * ux, dy + best * uy))
            })
            .collect()
    }

    /// Cabinet impact (diffraction) response: mean over drivers.
    pub fn cabinet_impact(&self, freq: &[f64]) -> Response {
        let r = self.axis_distance;
        let (h, v) = (self.angle_hor, self.angle_ver);
        // listener plus first-order reflections: the whole baffle turned toward the mirror image of the
        // listener (angle acos(R/d), d = √(R² + (2·offset)²), floor in the vertical plane, wall in the
        // horizontal one), at distance d, × 10^(−absorption/20)
        let loss = db_to_lin(-self.absorption_db);
        let image = |off: f64| {
            let d = r.hypot(2.0 * off);
            ((r / d).acos() * off.signum(), d)
        };
        let mut listeners = vec![(h, v, r, 1.0)];
        listeners.extend(self.floor_y.map(|y| image(self.mic.1 - y)).map(|(a, d)| (h, v + a, d, loss)));
        listeners.extend(self.wall_x.map(|x| image(self.mic.0 - x)).map(|(a, d)| (h + a, v, d, loss)));
        let mut values = vec![Complex64::default(); freq.len()];
        for &drv in &self.drivers {
            for &(lh, lv, d, g) in &listeners {
                for (v, p) in values.iter_mut().zip(self.driver_impact(drv, (lh, lv, d), freq)) {
                    *v += p * g / self.drivers.len() as f64;
                }
            }
        }
        Response::new("Cabinet impact", freq.to_vec(), values)
    }

    /// Response at an exported directivity angle. The export flips the sign of the baffle rotation
    /// on export, so `angle_deg` > 0 rotates the baffle by −angle (same convention as measurements).
    pub fn at_angle(&self, plane: Plane, angle_deg: f64, freq: &[f64]) -> Response {
        let mut b = self.clone();
        let a = (-angle_deg).to_radians();
        match plane {
            Plane::Hor => (b.angle_hor, b.angle_ver) = (a, 0.0),
            Plane::Ver => (b.angle_hor, b.angle_ver) = (0.0, a),
        }
        let mut r = b.cabinet_impact(freq);
        (r.plane, r.angle) = (plane, Some(angle_deg));
        r
    }

    /// Directivity set: horizontal 0…180° every `step`°, plus the
    /// vertical plane (±step…±(180−step)) and negative angles when asked.
    pub fn directivity(&self, step_deg: f64, vertical: bool, negative: bool, freq: &[f64]) -> Vec<Response> {
        let n = (180.0 / step_deg).floor() as i32;
        let mut out = vec![];
        for plane in [Plane::Hor, Plane::Ver] {
            if plane == Plane::Ver && !vertical {
                continue;
            }
            for i in (0..=n).filter(|&i| plane == Plane::Hor || (i > 0 && (i as f64 * step_deg) < 180.0)) {
                let a = i as f64 * step_deg;
                out.push(self.at_angle(plane, a, freq));
                if negative && i > 0 && a < 180.0 {
                    out.push(self.at_angle(plane, -a, freq));
                }
            }
        }
        out
    }

    /// Full space response : half-space response × cabinet impact / 2.
    pub fn full_space(&self, half_space: &Response) -> Response {
        let imp = self.cabinet_impact(&half_space.freq);
        let values = half_space.values.iter().zip(&imp.values).map(|(h, c)| h * c / 2.0).collect();
        Response::new(format!("{} full space", half_space.name), half_space.freq.clone(), values)
    }

    /// Response of one driver for the listener at distance `d` from the mic point, baffle turned by
    /// (`h`, `v`) [rad] around the mic point.
    fn driver_impact(&self, driver: (f64, f64), (h, v, d): (f64, f64, f64), freq: &[f64]) -> Vec<Complex64> {
        let r = self.axis_distance;
        let lp = (self.mic.0 - d * h.sin() * v.cos(), self.mic.1 - d * v.sin(), d * h.cos() * v.cos());
        let to_lp = |a: (f64, f64)| (a.0 - lp.0).hypot(a.1 - lp.1).hypot(lp.2);
        let rd = to_lp(driver);
        // off-axis angle: driver offset from the mic in the listener frame,
        // horizontal angle a = h + atan2(δx, d − δz), vertical b = v + atan2(δy, d − δz), θ = acos(cos a·cos b)
        // (equals the true angle unless both components are non-zero)
        let (px, py) = (driver.0 - self.mic.0, driver.1 - self.mic.1);
        let dl = (px * h.cos(), py * v.cos() - px * h.sin() * v.sin(), -py * v.sin() - px * h.sin() * v.cos());
        let (a, b) = (h + dl.0.atan2(d - dl.2), v + dl.1.atan2(d - dl.2));
        let cos_t = a.cos() * b.cos();
        let theta = cos_t.clamp(-1.0, 1.0).acos();
        let er = self.edge_radius;
        // (L, r/L) per ray, L = d + l − R/2
        let rays: Vec<(f64, f64)> = self
            .edge_points(driver)
            .into_iter()
            .map(|e| {
                let len = (e.0 - driver.0).hypot(e.1 - driver.1) + to_lp(e) - er / 2.0;
                (len, r / len)
            })
            .collect();
        // listener in the shadow of the baffle: the direct sound arrives over the shortest edge path (Keller's
        // geometrical theory of diffraction: a shadow field travels the shortest path around the obstacle)
        let direct_len = if cos_t < 0.0 && !self.open { rays.iter().map(|x| x.0).fold(f64::INFINITY, f64::min).max(rd) } else { rd };
        // angular weighting of the direct wave and of the edge sources: 1 + cos θ and cos θ
        let (direct_w, ray_w) = (1.0 + theta.cos(), theta.cos());
        // open baffle: rear wave (opposite polarity) diffracts over the same paths
        let rear = if self.open { 2.0 } else { 1.0 };
        // piston directivity: the direct wave leaves at θ off the axis, the edge sources lie in the baffle plane (90°)
        let (p_direct, p_rays) = (piston_response(freq, self.dd, theta.sin()), piston_response(freq, self.dd, 1.0));
        freq.iter()
            .enumerate()
            .map(|(i, &f)| {
                let k = 2.0 * PI * f / C_AIR;
                let edge = if er > 0.0 { (1.0 + (f * 8.0 * er / C_AIR).powf(2.5)).sqrt().recip() } else { 1.0 };
                let w = rear * ray_w * edge / rays.len() as f64;
                let diff: Complex64 = rays.iter().map(|&(len, g)| Complex64::from_polar(g * w, -k * (len - r))).sum();
                p_direct[i] * Complex64::from_polar(direct_w * r / direct_len, -k * (direct_len - r)) - p_rays[i] * diff
            })
            .collect()
    }
}
/// Rigid circular piston of diameter `dd` in an infinite baffle: far-field directivity 2·J1(ka·sinθ)/(ka·sinθ)
/// (Rayleigh) toward sin θ = `sin_t`, over `freq`. It is real: the sign change after each null is the physical 180°
/// phase step.
fn piston_response(freq: &[f64], dd: f64, sin_t: f64) -> Vec<Complex64> {
    freq.iter().map(|&f| Complex64::new(piston(2.0 * PI * f / C_AIR * dd / 2.0 * sin_t), 0.0)).collect()
}

/// Piston directivity 2·J1(x)/x.
fn piston(x: f64) -> f64 {
    if x < 1e-9 {
        return 1.0;
    }
    2.0 * j1(x) / x
}

/// Bessel J1 for x ≥ 0: power series below 12, Hankel asymptotic expansion above (|error| < 1e-10).
fn j1(x: f64) -> f64 {
    if x < 12.0 {
        // Σ (−1)^k (x/2)^(2k+1) / (k!(k+1)!)
        let (q, mut term, mut s) = (-(x * x) / 4.0, x / 2.0, x / 2.0);
        for k in 1..60 {
            term *= q / (k * (k + 1)) as f64;
            s += term;
            if term.abs() < 1e-17 * s.abs().max(1e-300) {
                break;
            }
        }
        s
    } else {
        // P, Q series with μ = 4ν² = 4
        let (mut p, mut q, mut tp) = (1.0, 0.0, 1.0);
        let z = 8.0 * x;
        for k in 1..12 {
            let odd = (2 * k - 1) as f64;
            tp *= (4.0 - odd * odd) / (k as f64 * z);
            if k % 2 == 1 {
                q += tp * if (k / 2) % 2 == 0 { 1.0 } else { -1.0 };
            } else {
                p += tp * if (k / 2) % 2 == 1 { -1.0 } else { 1.0 };
            }
        }
        let w = x - 0.75 * PI;
        (2.0 / (PI * x)).sqrt() * (p * w.cos() - q * w.sin())
    }
}
