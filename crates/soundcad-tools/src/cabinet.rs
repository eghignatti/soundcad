//! Cabinet interior: geometry, lining (damping) and internal resonances.
//!
//! A rectangular box: internal width and height come from the front minus the walls, the depth is an input.
//!
//! * surface `S = 2(WH + WD + HD)`; holes (driver cut-outs, vents) take their area away from what can be lined;
//! * lining of thickness `t` and density `ρ` on a share of the surface. Flow resistivity from the density with the
//!   Garai–Pompoli law for polyester fibre, `σ = 25.989·ρ^1.404` Pa·s/m² (Applied Acoustics 66, 2005; constants as
//!   quoted by Pelegrinis et al., SAPEM 2014; other fibres differ with fibre diameter); absorption of the layer on a
//!   rigid wall at normal incidence from the Delany–Bazley model (Applied Acoustics 3, 1970), used inside its range
//!   `0.01 < X = ρ₀f/σ < 1` (outside, X is held at the limit); bare walls absorb 0.03;
//! * the lining takes volume (net `Vb` = gross − lining − vents) and damps the box: `1/Qa = 1/60 + 0.6·r·σ/(σ+3000)`
//!   with `r` = lining volume / gross volume (empirical estimate, not a measurement);
//! * reverberation: Sabine `T60 = 0.161·V/(S·ᾱ)` (diffuse field; only a guide in a box this small at low frequency);
//! * internal modes `f = c/2·√((nx/W)² + (ny/H)² + (nz/D)²)` up to [`MODES_MAX_HZ`]. The pressure they build at the
//!   vent, relative to the uniform pressure of the lumped model, is
//!   `1 + Σ gain·(−ω²)/(ωn² − ω² + jω·ωn/Qn)` with `gain = ψ(driver)·ψ(vent)/Λ`. A vent on a pressure node of a
//!   mode does not radiate it. Each mode is damped by the walls (locally reacting, small admittance, Morse & Ingard):
//!   the decay constant is `δn = c·ᾱ(fn)·Σw εw·Sw/(8V)` over the three wall pairs, `εw = 2` if the mode varies along
//!   that axis, else 1, and `Qn = ωn/(2δn)`. Axial, tangential and oblique modes decay in the ratio 1 : 0.8 : 0.67 in
//!   a box with equal walls, as in the finite-element study of room decay times (COMSOL test bench, after Morse & Ingard).
//!
//! The cone is assumed not to feel the modes and the lining is not allowed to change the mode shapes.

use crate::enclosure::{Radiator, C_AIR, RHO};
use num_complex::Complex64;
use std::f64::consts::PI;

/// Highest mode frequency kept [Hz].
pub const MODES_MAX_HZ: f64 = 3000.0;
/// Absorption coefficient of a bare wall.
const ALPHA_WALL: f64 = 0.03;
/// Qa of a box with no lining.
const QA_EMPTY: f64 = 60.0;

/// What the user decides about the box interior.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Cabinet {
    /// Internal depth [mm].
    pub depth: f64,
    /// Wall thickness [mm].
    pub wall: f64,
    /// Lining thickness [mm].
    pub lining: f64,
    /// Lining density [kg/m³].
    pub density: f64,
    /// Lined share of the internal surface [%].
    pub coverage: f64,
}

impl Default for Cabinet {
    fn default() -> Self {
        Cabinet { depth: 250.0, wall: 18.0, lining: 20.0, density: 30.0, coverage: 50.0 }
    }
}

/// Where things are, from the project: front size, the holes in the box and the positions of driver and vent.
pub struct Setup {
    /// Front size [mm].
    pub width: f64,
    pub height: f64,
    /// Total area of driver cut-outs and vents [mm²].
    pub holes: f64,
    /// Volume taken by the vent tubes [l].
    pub vents_l: f64,
    /// Driver and vent centres on the front [mm]; no vent: `None`.
    pub driver: (f64, f64),
    pub vent: Option<(f64, f64)>,
}

/// One internal resonance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mode {
    pub n: (u32, u32, u32),
    pub f: f64,
    pub q: f64,
    /// Pressure shape at the vent (−1…1); 0 without a vent.
    pub at_vent: f64,
    /// `ψ(driver)·ψ(vent)/Λ`: how much the mode adds at the vent.
    pub gain: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Analysis {
    /// Internal width, height, depth [m].
    pub dims: (f64, f64, f64),
    pub gross_l: f64,
    pub lining_l: f64,
    pub vents_l: f64,
    /// Air volume [l] and loss factor to use in the box model.
    pub vb: f64,
    pub qa: f64,
    /// Areas [m²]: internal surface, holes, most that can be lined, lined.
    pub surface: f64,
    pub holes: f64,
    pub max_lined: f64,
    pub lined: f64,
    /// Estimated flow resistivity of the lining [Pa·s/m²].
    pub sigma: f64,
    thickness: f64,
    pub modes: Vec<Mode>,
}

/// Flow resistivity [Pa·s/m²] from density [kg/m³]: Garai–Pompoli law for polyester fibre.
pub fn flow_resistivity(density: f64) -> f64 {
    25.989 * density.max(1.0).powf(1.404)
}

/// Absorption coefficient of a porous layer of `d` [m] on a rigid wall, normal incidence (Delany–Bazley).
pub fn layer_alpha(sigma: f64, d: f64, f: f64) -> f64 {
    if d <= 0.0 {
        return ALPHA_WALL;
    }
    let x = (RHO * f / sigma).clamp(0.01, 1.0);
    let z0 = RHO * C_AIR;
    let zc = Complex64::new(1.0 + 0.0571 * x.powf(-0.754), -0.087 * x.powf(-0.732)) * z0;
    let k = Complex64::new(1.0 + 0.0978 * x.powf(-0.700), -0.189 * x.powf(-0.595)) * (2.0 * PI * f / C_AIR);
    let kd = k * d;
    let zs = Complex64::new(0.0, -1.0) * zc * kd.cos() / kd.sin();
    let r = (zs - z0) / (zs + z0);
    (1.0 - r.norm_sqr()).clamp(ALPHA_WALL, 1.0)
}

impl Cabinet {
    pub fn analyze(&self, s: &Setup) -> Analysis {
        let w = ((s.width - 2.0 * self.wall) * 1e-3).max(0.02);
        let h = ((s.height - 2.0 * self.wall) * 1e-3).max(0.02);
        let d = (self.depth * 1e-3).max(0.02);
        let surface = 2.0 * (w * h + w * d + h * d);
        let holes = (s.holes * 1e-6).min(surface);
        let max_lined = surface - holes;
        let lined = (self.coverage / 100.0 * surface).clamp(0.0, max_lined);
        let gross_l = w * h * d * 1e3;
        let thickness = (self.lining * 1e-3).clamp(0.0, 0.5 * w.min(h).min(d));
        let lining_l = (lined * thickness * 1e3).min(0.6 * gross_l);
        let vb = (gross_l - lining_l - s.vents_l).max(0.1);
        let sigma = flow_resistivity(self.density);
        let fill = lining_l / gross_l;
        let qa = 1.0 / (1.0 / QA_EMPTY + 0.6 * fill * sigma / (sigma + 3000.0));

        let mut a = Analysis { dims: (w, h, d), gross_l, lining_l, vents_l: s.vents_l, vb, qa, surface, holes, max_lined, lined, sigma, thickness, modes: vec![] };
        // pressure shapes are cosines on the box: positions relative to the internal corner
        let at = |p: (f64, f64)| (((p.0 - self.wall) * 1e-3).clamp(0.0, w), ((p.1 - self.wall) * 1e-3).clamp(0.0, h));
        let (dr, vt) = (at(s.driver), s.vent.map(at));
        let v = vb * 1e-3;
        let (nx, ny, nz) = ((2.0 * w * MODES_MAX_HZ / C_AIR) as u32, (2.0 * h * MODES_MAX_HZ / C_AIR) as u32, (2.0 * d * MODES_MAX_HZ / C_AIR) as u32);
        for i in 0..=nx {
            for j in 0..=ny {
                for k in 0..=nz {
                    if (i, j, k) == (0, 0, 0) {
                        continue;
                    }
                    let f = C_AIR / 2.0 * ((i as f64 / w).powi(2) + (j as f64 / h).powi(2) + (k as f64 / d).powi(2)).sqrt();
                    if f > MODES_MAX_HZ {
                        continue;
                    }
                    let psi = |p: (f64, f64)| (i as f64 * PI * p.0 / w).cos() * (j as f64 * PI * p.1 / h).cos();
                    let lambda = [i, j, k].iter().fold(1.0, |l, &n| if n > 0 { l * 0.5 } else { l });
                    let at_vent = vt.map_or(0.0, psi);
                    let gain = psi(dr) * at_vent / lambda;
                    // wall pairs this mode varies along take double damping
                    let eps = |n: u32| if n > 0 { 2.0 } else { 1.0 };
                    let wall_sum = eps(i) * 2.0 * h * d + eps(j) * 2.0 * w * d + eps(k) * 2.0 * w * h;
                    let delta = C_AIR * a.alpha(f) * wall_sum / (8.0 * v);
                    let q = (2.0 * PI * f / (2.0 * delta)).clamp(0.7, 300.0);
                    a.modes.push(Mode { n: (i, j, k), f, q, at_vent, gain });
                }
            }
        }
        a.modes.sort_by(|p, q| p.f.total_cmp(&q.f));
        a
    }
}

impl Analysis {
    /// Mean absorption coefficient of the walls at `f`.
    pub fn alpha(&self, f: f64) -> f64 {
        let walls = (self.surface - self.holes).max(1e-3);
        let lined = layer_alpha(self.sigma, self.thickness, f);
        (self.lined * lined + (walls - self.lined) * ALPHA_WALL) / walls
    }

    /// `r` with the air volume and Qa of this analysis (an infinite baffle has no box).
    pub fn fit(&self, r: Radiator) -> Radiator {
        match r {
            Radiator::Closed { ql, .. } => Radiator::Closed { vb: self.vb, qa: self.qa, ql },
            Radiator::BassReflex { ql, diam, len, qp, n, end_corr, .. } => Radiator::BassReflex { vb: self.vb, qa: self.qa, ql, diam, len, qp, n, end_corr },
            Radiator::Passive { ql, pr, .. } => Radiator::Passive { vb: self.vb, qa: self.qa, ql, pr },
            Radiator::InfiniteBaffle => r,
        }
    }

    /// Reverberation time [s] (Sabine) at `f`.
    pub fn t60(&self, f: f64) -> f64 {
        0.161 * self.vb * 1e-3 / ((self.surface - self.holes).max(1e-3) * self.alpha(f))
    }

    /// Pressure at the vent relative to the lumped model (1 = no internal modes).
    pub fn vent_factor(&self, f: f64) -> Complex64 {
        vent_factor(&self.modes, f)
    }
}

/// See [`Analysis::vent_factor`].
pub fn vent_factor(modes: &[Mode], f: f64) -> Complex64 {
    let w = 2.0 * PI * f;
    modes.iter().fold(Complex64::new(1.0, 0.0), |acc, m| {
        let wn = 2.0 * PI * m.f;
        acc + m.gain * (-w * w) / Complex64::new(wn * wn - w * w, w * wn / m.q)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn setup() -> Setup {
        Setup { width: 336.0, height: 436.0, holes: 20000.0, vents_l: 0.5, driver: (168.0, 300.0), vent: Some((168.0, 90.0)) }
    }

    #[test]
    fn first_axial_mode_matches_depth() {
        let c = Cabinet { depth: 500.0, wall: 18.0, ..Default::default() };
        let a = c.analyze(&setup());
        // W = 0.3 m, H = 0.4 m, D = 0.5 m: lowest mode is the depth axial c/(2D)
        assert!((a.modes[0].f - C_AIR / 1.0).abs() < 1.0, "{}", a.modes[0].f);
        assert_eq!(a.modes[0].n, (0, 0, 1));
    }

    #[test]
    fn lining_absorbs_more_at_high_frequency_and_never_exceeds_free_surface() {
        let c = Cabinet { coverage: 500.0, ..Default::default() };
        let a = c.analyze(&setup());
        assert!(a.lined <= a.max_lined + 1e-12);
        assert!(a.alpha(2000.0) > a.alpha(100.0));
        // 25 mm of 30 kg/m³ polyester on a rigid wall: about 0.26 at 1 kHz, 0.47 at 2 kHz, little at 250 Hz
        let al = |f| layer_alpha(flow_resistivity(30.0), 0.025, f);
        assert!(al(250.0) < 0.1 && (0.15..0.45).contains(&al(1000.0)) && al(2000.0) > 0.4);
        assert!(a.qa < QA_EMPTY && a.vb < a.gross_l);
    }

    #[test]
    fn mode_decay_ratio_axial_tangential_oblique() {
        // cube, equal bare walls: decay constants 1 : 1.25 : 1.5 (decay times 3 : 2.4 : 2 in the COMSOL study)
        let c = Cabinet { depth: 400.0, wall: 0.0, coverage: 0.0, ..Default::default() };
        let a = c.analyze(&Setup { width: 400.0, height: 400.0, holes: 0.0, vents_l: 0.0, driver: (0.0, 0.0), vent: None });
        let delta = |n| {
            let m = a.modes.iter().find(|m| m.n == n).unwrap();
            2.0 * PI * m.f / (2.0 * m.q)
        };
        assert!((delta((1, 1, 0)) / delta((1, 0, 0)) - 1.25).abs() < 1e-9);
        assert!((delta((1, 1, 1)) / delta((1, 0, 0)) - 1.5).abs() < 1e-9);
    }

    #[test]
    fn flow_resistivity_garai_pompoli() {
        // 30 kg/m³ polyester: about 3 kPa·s/m²
        assert!((flow_resistivity(30.0) - 3082.0).abs() < 20.0, "{}", flow_resistivity(30.0));
    }

    #[test]
    fn no_lining_leaves_the_lumped_box() {
        let a = Cabinet { coverage: 0.0, ..Default::default() }.analyze(&setup());
        assert_eq!(a.lining_l, 0.0);
        assert!((a.qa - QA_EMPTY).abs() < 1e-9);
        assert!((a.vent_factor(10.0) - Complex64::new(1.0, 0.0)).norm() < 0.01);
    }

    #[test]
    fn vent_on_a_node_does_not_radiate_the_mode() {
        // depth mode (0,0,1): the vent and the driver are on the front, so both see it; mode (1,0,0) has a node
        // at the middle of the width, where the vent is
        let a = Cabinet::default().analyze(&setup());
        let m = a.modes.iter().find(|m| m.n == (1, 0, 0)).unwrap();
        assert!(m.at_vent.abs() < 1e-6, "{}", m.at_vent);
    }

    #[test]
    fn modes_change_the_port_only_around_their_frequency() {
        use crate::enclosure::{Driver, Enclosure};
        let driver = Driver { manufacturer: "t".into(), model: "t".into(), re: 5.5, le: 0.5, leb: 0.0, ke: 0.0, rss: 0.0, rms: 1.5, mms: 20.0, cms: 0.6, sd: 85.0, bl: 7.0 };
        let c = Cabinet { depth: 400.0, coverage: 0.0, ..Default::default() };
        let a = c.analyze(&Setup { vent: Some((60.0, 90.0)), ..setup() });
        let r = a.fit(Radiator::BassReflex { vb: 0.0, qa: 0.0, ql: 15.0, diam: 5.0, len: 12.0, qp: 100.0, n: 1, end_corr: 0.85 });
        let freq = [40.0, 60.0, a.modes[0].f];
        let flat = Enclosure { driver: driver.clone(), radiator: r, eg: 2.83, modes: vec![] }.simulate(&freq).port_spl_2pi.unwrap();
        let with = Enclosure { driver, radiator: r, eg: 2.83, modes: a.modes.clone() }.simulate(&freq).port_spl_2pi.unwrap();
        let ratio = |i: usize| with.values[i].norm() / flat.values[i].norm();
        assert!((ratio(0) - 1.0).abs() < 0.02 && (ratio(1) - 1.0).abs() < 0.02, "{} {}", ratio(0), ratio(1));
        assert!(ratio(2) > 1.5, "{}", ratio(2));
    }
}
