//! Driver acoustics and summing of the crossover: filter voltage × measured response at the
//! listening angle × distance (1/r and delay), summed over drivers.
//!
//! Model:
//! * driver SPL = filter response (V across driver / Eg) × measured response (dB SPL at 2.83 V
//!   scale) × `Rref/dist` × `e^{−jω(dist−Rref)/c}`, c = 344 m/s;
//! * `dist` from the listening point (on the axis at `Rref` from the design origin) to the driver
//!   position (X, Y, Z), Z positive away from the listener;
//! * off-axis response between two measured angles: interpolated linearly in the angle (log magnitude, phase
//!   along the shortest arc), at the exact angle (see `DriverResponses::toward`);
//! * without vertical measurements the horizontal ones are used for the vertical angle.

use crate::circuit::{solve, Solution};
use crate::project::{Part, Project};
use num_complex::Complex64;
use std::collections::HashMap;
use std::f64::consts::PI;
use soundcad_core::response::{db_to_lin, unwrap};
use soundcad_core::{Response, C_AIR};

/// Measured responses of one driver: horizontal plane and vertical plane, sorted by angle.
/// The vertical set always contains the horizontal 0° response.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct DriverResponses {
    pub hor: Vec<(f64, Response)>,
    pub ver: Vec<(f64, Response)>,
    pub impedance: Option<Response>,
}

/// Weighted mean of complex values: log magnitude averaged with the weights, phase = phase of the first value plus
/// the weighted mean of the (−180°, 180°] differences to it, i.e. interpolation along the shortest arc. Weights are
/// normalised.
fn blend(parts: impl Iterator<Item = (f64, Complex64)>) -> Complex64 {
    let (mut wsum, mut ln_mag, mut dphase, mut first) = (0.0, 0.0, 0.0, None::<f64>);
    for (w, v) in parts {
        let p = v.arg();
        let p0 = *first.get_or_insert(p);
        wsum += w;
        ln_mag += w * v.norm().max(1e-12).ln();
        dphase += w * (p - p0 + PI).rem_euclid(2.0 * PI) - w * PI;
    }
    Complex64::from_polar((ln_mag / wsum).exp(), first.unwrap_or(0.0) + dphase / wsum)
}

/// Measurements of `set` enclosing `angle` (nearest at or below and at or above) with linear weights.
fn bracket(set: &[(f64, Response)], angle: f64) -> Option<Vec<(f64, &Response)>> {
    let lo = set.iter().rev().find(|x| x.0 <= angle)?;
    let hi = set.iter().find(|x| x.0 >= angle)?;
    if hi.0 == lo.0 {
        return Some(vec![(1.0, &lo.1)]);
    }
    let t = (angle - lo.0) / (hi.0 - lo.0);
    Some(vec![(1.0 - t, &lo.1), (t, &hi.1)])
}
impl DriverResponses {
    /// The two sets sorted by angle (measurements at the same angle keep their order).
    pub fn new(mut hor: Vec<(f64, Response)>, mut ver: Vec<(f64, Response)>, impedance: Option<Response>) -> Self {
        hor.sort_by(|a, b| a.0.total_cmp(&b.0));
        ver.sort_by(|a, b| a.0.total_cmp(&b.0));
        Self { hor, ver, impedance }
    }

    /// Measured responses (weight, response) for the plane component at signed `angle`: the enclosing
    /// measurements of that plane, else at −angle, else the other plane at ±angle (so one-sided or
    /// missing data is mirrored / borrowed).
    fn component(&self, vertical: bool, angle: f64) -> Option<Vec<(f64, &Response)>> {
        let (own, other) = if vertical { (&self.ver, &self.hor) } else { (&self.hor, &self.ver) };
        bracket(own, angle)
            .or_else(|| bracket(own, -angle))
            .or_else(|| bracket(other, angle))
            .or_else(|| bracket(other, -angle))
            // outside every measured range (e.g. files only to ±40°): the closest measured angle, as
            // does without "Interpolate"; never an empty result
            .or_else(|| {
                let closest = own.iter().chain(other.iter()).min_by(|a, b| (a.0 - angle).abs().total_cmp(&(b.0 - angle).abs()))?;
                Some(vec![(1.0, &closest.1)])
            })
    }

    /// Response toward off-axis angle `theta` (degrees, any real value) at azimuth φ (cos φ, sin φ) around the
    /// driver axis. The two measured planes are the φ = 0 and φ = 90° cuts; in between the response is
    /// `cos²φ · H_hor(θ) + sin²φ · H_ver(θ)`, the lowest-order azimuthal harmonic that matches both cuts (θ taken on
    /// the side of each plane where the direction lies). Within a plane the response between two measured angles
    /// is interpolated linearly in the angle. Magnitudes are combined in dB (log magnitude, the usual domain for
    /// acoustic levels) and phases along the shortest arc from the first component (see `blend`).
    pub fn toward(&self, theta: f64, cos_phi: f64, sin_phi: f64) -> Vec<Complex64> {
        let sign = |x: f64| if x > 0.0 { 1.0 } else if x < 0.0 { -1.0 } else { 0.0 };
        let c2 = cos_phi * cos_phi;
        let mut parts = vec![];
        for (vertical, w, a) in [(false, c2, theta * sign(cos_phi)), (true, 1.0 - c2, theta * sign(sin_phi))] {
            if let Some(b) = self.component(vertical, a) {
                parts.extend(b.into_iter().map(|(t, r)| (t * w, r)));
            }
        }
        (0..self.hor[0].1.len()).map(|i| blend(parts.iter().map(|(w, r)| (*w, r.values[i])))).collect()
    }
    pub fn freq(&self) -> Option<&[f64]> {
        self.hor.first().map(|x| x.1.freq.as_slice())
    }
}
/// Placement and settings of one driver instance.
#[derive(Clone, Debug)]
struct Placed {
    /// index into `meas`
    di: usize,
    /// index of the circuit driver whose voltage feeds this source (itself, or the driver a port belongs to)
    vj: usize,
    x: f64,
    y: f64,
    z: f64,
    gain: f64,
    delay_us: f64,
    /// part params R (rotation, deg) and T (tilt, deg)
    rot: f64,
    tilt: f64,
}

/// Everything the crossover tab shows.
pub struct Simulation {
    pub freq: Vec<f64>,
    pub circuit: Solution,
    /// Filter response (V across the driver / Eg) per driver, in schematic order.
    pub filter: Vec<Response>,
    /// Driver SPL at the listening point per driver (filter included).
    pub driver_spl: Vec<Response>,
    /// Sum of all drivers.
    pub total: Response,
    meas: Vec<DriverResponses>,
    /// unwrapped phase of each driver's impedance (empty without one), for `driver_impedance`
    z_phase: Vec<Vec<f64>>,
    placed: Vec<Placed>,
    rref: f64,
    /// Rotation of the simulated orbits around Z [deg]
    plane_rot: f64,
    /// Power response and DI settings (defaults; set after `simulate`).
    pub opts: PowerOptions,
}

/// CTA-2034-A power weights per 10° step, 0…180° (applied to every measurement at that |angle|).
const CTA_WEIGHTS: [f64; 19] = [
    0.000604486, 0.004730189, 0.008955027, 0.012387354, 0.014989611, 0.016868154, 0.018165963, 0.019006744, 0.019477786, 0.019629372,
    0.019477786, 0.019006744, 0.018165963, 0.016868154, 0.014989611, 0.012387354, 0.008955027, 0.004730189, 0.000604486,
];

impl Simulation {
    /// Sound pressure of driver `j` at the listening direction (`hor`, `ver` in degrees; one of them is
    /// 0 for the standard planes), 2.83 V-scaled responses × filter × distance and delay.
    pub fn driver_pressure(&self, j: usize, hor: f64, ver: f64) -> Vec<Complex64> {
        self.driver_pressure_r(j, hor, ver, self.rref)
    }

    /// As `driver_pressure` with the listener on a sphere of radius `r` mm (level and delay still
    /// relative to `rref`).
    fn driver_pressure_r(&self, j: usize, hor: f64, ver: f64, r: f64) -> Vec<Complex64> {
        let mut g = self.source_gain_r(j, hor, ver, r);
        for (g, v) in g.iter_mut().zip(&self.circuit.driver_v[self.placed[j].vj]) {
            *g *= v;
        }
        g
    }

    /// Pressure of source `j` toward (`hor`, `ver`) per volt across its circuit driver: everything of
    /// `driver_pressure` except the filter, so the filter can change without redoing the acoustics.
    pub fn source_gain(&self, j: usize, hor: f64, ver: f64) -> Vec<Complex64> {
        self.source_gain_r(j, hor, ver, self.rref)
    }

    /// Number of acoustic sources (drivers, then coupled sources) and the circuit driver feeding source `j`.
    pub fn source_count(&self) -> usize {
        self.placed.len()
    }

    pub fn source_feeder(&self, j: usize) -> usize {
        self.placed[j].vj
    }

    fn source_gain_r(&self, j: usize, hor: f64, ver: f64, r: f64) -> Vec<Complex64> {
        let pl = &self.placed[j];
        // listener on a sphere of radius r around the design origin; z toward the listener is −z
        let (ah, av) = (hor.to_radians(), ver.to_radians());
        let l = (r * ah.sin() * av.cos(), r * av.sin(), -r * ah.cos() * av.cos());
        let (sp, cp) = self.plane_rot.to_radians().sin_cos();
        let l = (l.0 * cp - l.1 * sp, l.0 * sp + l.1 * cp, l.2);
        let d = (l.0 - pl.x, l.1 - pl.y, l.2 - pl.z);
        // into the driver frame: undo rotation R (around Y) and tilt T (around X)
        let (sa, ca) = pl.rot.to_radians().sin_cos();
        let d = (d.0 * ca + d.2 * sa, d.1, -d.0 * sa + d.2 * ca);
        let (sb, cb) = (-pl.tilt).to_radians().sin_cos();
        let d = (d.0, d.1 * cb - d.2 * sb, d.1 * sb + d.2 * cb);
        let dist = (d.0 * d.0 + d.1 * d.1 + d.2 * d.2).sqrt();
        let depth = -d.2;
        // angle off the driver axis and azimuth of the offset
        let off = (d.0 * d.0 + d.1 * d.1).sqrt();
        let theta = off.atan2(depth).to_degrees();
        let (cphi, sphi) = if off == 0.0 { (1.0, 0.0) } else { (d.0 / off, d.1 / off) };
        let resp = self.meas[pl.di].toward(theta, cphi, sphi);
        // level and delay reference: the listening distance (1/r law relative to the reference distance)
        let dref = self.rref;
        (0..self.freq.len())
            .map(|i| {
                let delay = Complex64::from_polar(dref / dist, -2.0 * PI * self.freq[i] * ((dist - dref) * 1e-3 / C_AIR + pl.delay_us * 1e-6));
                resp[i] * delay * pl.gain
            })
            .collect()
    }

    /// Adds an acoustic source fed by circuit driver `j` (e.g. its bass-reflex port): responses `meas`
    /// at position `pos` (mm, like driver X/Y/Z), same polarity and gain as the driver. Updates `total`.
    pub fn add_coupled_source(&mut self, j: usize, meas: DriverResponses, pos: (f64, f64, f64)) {
        let base = self.placed[j].clone();
        self.meas.push(meas);
        self.placed.push(Placed { di: self.meas.len() - 1, x: pos.0, y: pos.1, z: pos.2, rot: 0.0, tilt: 0.0, delay_us: 0.0, ..base });
        self.total = Response::new("Total SPL", self.freq.clone(), self.pressure(0.0, 0.0));
    }

    /// On-axis pressure of each coupled source (bass-reflex vent …) with the index of the circuit driver feeding it.
    pub fn coupled_sources(&self) -> Vec<(usize, Vec<Complex64>)> {
        (self.circuit.driver_parts.len()..self.placed.len()).map(|k| (self.placed[k].vj, self.driver_pressure(k, 0.0, 0.0))).collect()
    }

    /// Sound pressure summed over all drivers.
    pub fn pressure(&self, hor: f64, ver: f64) -> Vec<Complex64> {
        self.pressure_r(hor, ver, self.rref)
    }

    fn pressure_r(&self, hor: f64, ver: f64, r: f64) -> Vec<Complex64> {
        let mut total = vec![Complex64::new(0.0, 0.0); self.freq.len()];
        for j in 0..self.placed.len() {
            for (t, v) in total.iter_mut().zip(self.driver_pressure_r(j, hor, ver, r)) {
                *t += v;
            }
        }
        total
    }

    /// Response with 1st-order room reflections (image-source method): on-axis pressure plus, per enabled
    /// surface, the whole speaker's pressure toward the listener's mirror image, at the exact direction of the image,
    /// listener on a sphere through the image (path length), × 10^(−absorption/20).
    /// Floor/ceiling: path √(R² + (2y)²), ver = acos(R/path)·sign(y). Walls (toe-in τ rotates the
    /// speaker away from the left wall): image (x, z) = (R sin τ − 2X, R cos τ) for the left wall,
    /// (R sin τ, R cos τ + 2Z) for the front wall, hor = −τ − atan2(x, z) resp.
    /// 180 − τ − atan2(x, z), wrapped to ±180.
    pub fn room_response(&self, room: &crate::project::Room) -> Response {
        let r = self.rref;
        let mut paths: Vec<(f64, f64, f64, f64)> = vec![]; // hor, ver, radius, absorption
        for (on, y, a) in [(room.floor, room.floor_y, room.absorp_floor), (room.ceiling, room.ceiling_y, room.absorp_ceil)] {
            if on {
                let path = r.hypot(2.0 * y);
                paths.push((0.0, (r / path).acos().to_degrees() * y.signum(), path, a));
            }
        }
        let wrap = |a: f64| (a + 180.0).rem_euclid(360.0) - 180.0;
        let tau = room.toe_in;
        let (st, ct) = (r * tau.to_radians().sin(), r * tau.to_radians().cos());
        if room.left_wall {
            let (x, z) = (st - 2.0 * room.left_wall_x, ct);
            paths.push((wrap(-tau - x.atan2(z).to_degrees()), 0.0, x.hypot(z), room.absorp_wall));
        }
        if room.front_wall {
            let (x, z) = (st, ct + 2.0 * room.front_wall_z);
            paths.push((wrap(180.0 - tau - x.atan2(z).to_degrees()), 0.0, x.hypot(z), room.absorp_wall));
        }
        let mut total = self.pressure(0.0, 0.0);
        for (h, v, path, a) in paths {
            let g = db_to_lin(-a);
            for (t, p) in total.iter_mut().zip(self.pressure_r(h, v, path)) {
                *t += p * g;
            }
        }
        Response::new("Room response", self.freq.clone(), total)
    }
    /// CTA-2034-A curves; every direction is computed once (see `Cta`).
    pub fn cta(&self) -> Cta {
        let rings = self.rings();
        let mut p: HashMap<(i32, i32), Vec<Complex64>> = HashMap::new();
        for (h, v) in LW_DIRS.iter().copied().chain(rings.iter().map(|r| (r.0, r.1))) {
            p.entry((h as i32, v as i32)).or_insert_with(|| self.pressure(h, v));
        }
        let at = |h: f64, v: f64, i: usize| p[&(h as i32, v as i32)][i];
        let n = self.freq.len();
        let (mut lw, mut er, mut sp, mut pir) = (Vec::with_capacity(n), Vec::with_capacity(n), Vec::with_capacity(n), Vec::with_capacity(n));
        for i in 0..n {
            let lw_ms = LW_DIRS.iter().map(|&(h, v)| at(h, v, i).norm_sqr()).sum::<f64>() / LW_DIRS.len() as f64;
            let mean: Complex64 = LW_DIRS.iter().map(|&(h, v)| at(h, v, i)).sum();
            let (sp_ms, er_ms) = ring_sums(rings.iter().map(|&(h, v, w, b)| (w, b, at(h, v, i).norm_sqr())));
            lw.push(Complex64::from_polar(lw_ms.sqrt(), mean.arg()));
            er.push(Complex64::new(er_ms.sqrt(), 0.0));
            sp.push(Complex64::new(sp_ms.sqrt(), 0.0));
            pir.push(Complex64::new(in_room_ms(lw_ms, er_ms, sp_ms).sqrt(), 0.0));
        }
        // DI: +3.01 dB in half space and +6.02 dB in corner
        let k = if self.opts.corner { 2.0 } else if self.opts.half_space { 2f64.sqrt() } else { 1.0 };
        let index = |name: &str, den: &[Complex64]| Response::new(name, self.freq.clone(), lw.iter().zip(den).map(|(r, d)| Complex64::new(k * r.norm() / d.norm(), 0.0)).collect());
        let (di, er_di) = (index("Directivity index", &sp), index("Early reflections DI", &er));
        let curve = |name: &str, v: Vec<Complex64>| Response::new(name, self.freq.clone(), v);
        Cta { di, er_di, listening_window: curve("Listening window", lw), early_reflections: curve("Early reflections", er), power: curve("Power response", sp), in_room: curve("In-room response", pir) }
    }

    /// Directions (hor, ver, power weight, early-reflection bounce) of the horizontal and vertical rings
    /// (10° step, −180…180 in both, so 0 and ±180 appear twice), summed by `ring_sums`:
    /// half space keeps |angle| ≤ 90 in both rings, corner |hor| ≤ 45 and |ver| ≤ 90; weight = CTA-2034-A
    /// table, or ∫sin over ±5° (sphere), or constant (cylinder); −180 has no power weight.
    /// Bounces: 0 front |h| ≤ 30, 1 side 40…80, 2 rear ≥ 90, 3 floor ver −40…−20, 4 ceiling ver 40…60.
    pub fn rings(&self) -> Vec<(f64, f64, f64, Option<usize>)> {
        let o = &self.opts;
        let mut out = vec![];
        for plane in 0..2 {
            for a in (-180..=180i32).step_by(10) {
                let lim = if plane == 0 && o.corner { 45 } else if o.corner || o.half_space { 90 } else { 180 };
                if a.abs() > lim {
                    continue;
                }
                let w = if a == -180 {
                    0.0
                } else if !o.sphere {
                    10.0
                } else if o.cta_weights {
                    CTA_WEIGHTS[(a.abs() / 10) as usize]
                } else {
                    let (lo, hi) = ((a as f64 - 5.0).max(-180.0).to_radians(), (a as f64 + 5.0).min(180.0).to_radians());
                    if lo < 0.0 && hi > 0.0 { 1.0 - ((hi - lo) / 2.0).cos() } else { (hi.cos() - lo.cos()).abs() }
                };
                let bounce = if plane == 0 {
                    match a.abs() {
                        0..=30 => Some(0),
                        40..=80 => Some(1),
                        x if x >= 90 => Some(2),
                        _ => None,
                    }
                } else {
                    match a {
                        -40..=-20 => Some(3),
                        40..=60 => Some(4),
                        _ => None,
                    }
                };
                let (h, v) = if plane == 0 { (a as f64, 0.0) } else { (0.0, a as f64) };
                out.push((h, v, w, bounce));
            }
        }
        out
    }

    /// Solves the circuit of `p` again (same drivers and responses, other component values) on `freq`.
    pub fn resolve(&self, p: &Project, freq: &[f64]) -> Result<Solution, String> {
        solve(p, freq, &|part: &Part, f: f64| driver_impedance(p, &self.meas, &self.z_phase, part, f))
    }
}

/// CTA-2034-A curves (magnitude only, but the listening window has the phase of the complex mean):
/// listening window = RMS of `LW_DIRS`; early reflections = RMS of the five bounces (floor, ceiling, front, side
/// and rear wall), each the RMS of its ring directions; sound power = weighted mean square over the rings;
/// predicted in-room (`in_room_ms`); DI = listening window over sound power, ER DI = over early reflections
/// (linear ratios).
pub struct Cta {
    pub listening_window: Response,
    pub early_reflections: Response,
    pub power: Response,
    pub in_room: Response,
    pub di: Response,
    pub er_di: Response,
}

/// Listening window directions (hor, ver): on axis, ±10/20/30° horizontal and ±10° vertical.
pub const LW_DIRS: [(f64, f64); 9] = [(0.0, 0.0), (10.0, 0.0), (-10.0, 0.0), (20.0, 0.0), (-20.0, 0.0), (30.0, 0.0), (-30.0, 0.0), (0.0, 10.0), (0.0, -10.0)];

/// Sound power and early reflections (mean squares) at one frequency from (power weight, bounce, mean square)
/// of every ring direction (`Simulation::rings`).
pub fn ring_sums(dirs: impl Iterator<Item = (f64, Option<usize>, f64)>) -> (f64, f64) {
    let (mut sp, mut wsum, mut er, mut cnt) = (0.0, 0.0, [0.0f64; 5], [0usize; 5]);
    for (w, b, ms) in dirs {
        sp += w * ms;
        wsum += w;
        if let Some(b) = b {
            er[b] += ms;
            cnt[b] += 1;
        }
    }
    (sp / wsum, (0..5).filter(|&b| cnt[b] > 0).map(|b| er[b] / cnt[b] as f64).sum::<f64>() / 5.0)
}

/// Predicted in-room response (mean square): 12 % listening window, 44 % early reflections, 44 % sound power.
pub fn in_room_ms(lw: f64, er: f64, sp: f64) -> f64 {
    0.12 * lw + 0.44 * er + 0.44 * sp
}

/// Power response and DI settings.
#[derive(Clone, Copy, Debug)]
pub struct PowerOptions {
    /// intensity on spherical surface (false: cylinder)
    pub sphere: bool,
    pub half_space: bool,
    pub corner: bool,
    pub cta_weights: bool,
}

impl Default for PowerOptions {
    fn default() -> Self {
        Self { sphere: true, half_space: false, corner: false, cta_weights: true }
    }
}
/// Group delay [s] as shown: τ = −dφ/dω from central differences of the unwrapped phase (one-sided at the ends),
/// then a plain moving average over a window of 1/3 octave (±1/6 octave around each point), which takes the
/// noise out of the numerical derivative.
pub fn group_delay(r: &Response) -> Vec<f64> {
    let raw = group_delay_raw(r);
    let half = 2f64.powf(1.0 / 6.0);
    r.freq
        .iter()
        .map(|&f| {
            let (lo, hi) = (r.freq.partition_point(|&x| x < f / half), r.freq.partition_point(|&x| x <= f * half));
            raw[lo..hi].iter().sum::<f64>() / (hi - lo) as f64
        })
        .collect()
}

/// Impedance of a Driver part: the measured one of its driver (unwrapped phase in `z_phase`) × scale, else the
/// nominal Z; 8 Ω for a model not in the drivers.
fn driver_impedance(p: &Project, meas: &[DriverResponses], z_phase: &[Vec<f64>], part: &Part, f: f64) -> Complex64 {
    let Some(di) = p.drivers.iter().position(|d| d.model == part.model) else { return Complex64::new(8.0, 0.0) };
    match &meas[di].impedance {
        Some(r) => r.sample_at(f, &z_phase[di]) * p.drivers[di].impedance_scale,
        None => Complex64::new(p.drivers[di].z, 0.0),
    }
}

/// Runs the crossover with the responses in memory: `meas[i]` belongs to `p.drivers[i]`.
pub fn simulate_with(p: &Project, meas: Vec<DriverResponses>, rref: f64) -> Result<Simulation, String> {
    let freq = meas.iter().find_map(|m| m.freq().map(|f| f.to_vec())).ok_or("no driver responses")?;
    let driver_index = |part: &Part| p.drivers.iter().position(|d| d.model == part.model);
    let z_phase: Vec<Vec<f64>> = meas.iter().map(|m| m.impedance.as_ref().map_or(vec![], |r| unwrap(&r.values))).collect();
    let circuit = solve(p, &freq, &|part: &Part, f: f64| driver_impedance(p, &meas, &z_phase, part, f))?;
    let mut sim = Simulation {
        freq: freq.clone(),
        filter: vec![],
        driver_spl: vec![],
        total: Response::new("Total SPL", freq.clone(), vec![Complex64::new(0.0, 0.0); freq.len()]),
        circuit,
        meas,
        z_phase,
        placed: vec![],
        rref,
        plane_rot: p.room.plane_rotation,
        opts: PowerOptions::default(),
    };
    for (j, &k) in sim.circuit.driver_parts.clone().iter().enumerate() {
        let part = &p.parts[k];
        let di = driver_index(part).ok_or_else(|| format!("driver model {} not in the project drivers", part.model))?;
        let sign = if part.inverted != p.drivers[di].invert { -1.0 } else { 1.0 };
        let mute = if part.muted || p.drivers[di].mute { 0.0 } else { 1.0 };
        sim.filter.push(Response::new(format!("{} filter", part.id), freq.clone(), sim.circuit.driver_v[j].clone()));
        sim.placed.push(Placed {
            di,
            vj: j,
            x: part.param("X").unwrap_or(0.0) + p.room.offset_x,
            y: part.param("Y").unwrap_or(0.0) + p.room.offset_y,
            z: part.param("Z").unwrap_or(0.0),
            gain: sign * mute * p.drivers[di].scale,
            delay_us: p.drivers[di].delay_us,
            rot: part.param("R").unwrap_or(0.0),
            tilt: part.param("T").unwrap_or(0.0),
        });
    }
    for j in 0..sim.placed.len() {
        let v = sim.driver_pressure(j, 0.0, 0.0);
        sim.driver_spl.push(Response::new(format!("{} SPL", p.parts[sim.circuit.driver_parts[j]].id), freq.clone(), v));
    }
    sim.total = Response::new("Total SPL", freq.clone(), sim.pressure(0.0, 0.0));
    Ok(sim)
}
/// −dφ/dω by central differences of the unwrapped phase (one-sided at the ends), in seconds.
fn group_delay_raw(r: &Response) -> Vec<f64> {
    let ph = unwrap(&r.values);
    let n = r.len();
    (0..n)
        .map(|i| {
            let (a, b) = (i.saturating_sub(1), (i + 1).min(n - 1));
            if a == b { 0.0 } else { -(ph[b] - ph[a]) / (2.0 * PI * (r.freq[b] - r.freq[a])) }
        })
        .collect()
}
