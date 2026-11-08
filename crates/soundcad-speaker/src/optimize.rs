//! Crossover optimizer: adjusts the values of the selected passive parts so the
//! simulated reference-axis or listening-window response (and optionally sound power or predicted in-room
//! response) follows a target curve.
//!
//! The acoustics of every source (driver, vent …) toward each evaluated direction do not depend on the
//! filter, so they are computed once (`Simulation::source_gain`); one evaluation then only solves the
//! circuit on the frequency points inside the range and sums `Σ V_j · G_j` per direction.
//!
//! Error = mean squared dB difference to the target over the range (level ignored unless `seek_level`),
//! shared between the main and the second target by percent, plus penalty terms (minimum impedance,
//! maximum filter gain, reference-axis linearity). Solver: Nelder–Mead on the logarithm of the values,
//! restarted until it stops improving. Values can be rounded to an E-series afterwards.

use num_complex::Complex64;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use soundcad_core::Response;
use soundcad_xover::sim::{in_room_ms, ring_sums, LW_DIRS};
use soundcad_xover::{PartKind, Project, Simulation};

use crate::schematic::{step, Snap};
use crate::Speaker;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Measure {
    RefAxis,
    ListeningWindow,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Slope {
    Butterworth,
    LinkwitzRiley,
}

/// A high-pass or low-pass edge of the target (LR order = total order, even).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Edge {
    pub slope: Slope,
    pub order: u32,
    pub fc: f64,
}

impl Edge {
    fn db(&self, f: f64, high_pass: bool) -> f64 {
        let x = if high_pass { self.fc / f } else { f / self.fc };
        match self.slope {
            Slope::Butterworth => -10.0 * (1.0 + x.powi(2 * self.order as i32)).log10(),
            Slope::LinkwitzRiley => -20.0 * (1.0 + x.powi(self.order as i32)).log10(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Secondary {
    Power,
    InRoom,
}

/// Second response optimized together with the main one (e.g. from the CTA-2034 chart): its target is the
/// main target plus `level` dB and `tilt` dB/octave; `weight` is its share of the error (0…1).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SecondaryTarget {
    pub kind: Secondary,
    pub weight: f64,
    pub level: f64,
    pub tilt: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    pub measure: Measure,
    /// Target level at 1 kHz [dB SPL], tilt [dB/octave] about 1 kHz, optional edges.
    pub level: f64,
    pub tilt: f64,
    pub hp: Option<Edge>,
    pub lp: Option<Edge>,
    /// Target read from a response file instead (dB, interpolated on log frequency), shifted by `file_offset` dB.
    pub target_file: Option<Response>,
    pub file_offset: f64,
    pub secondary: Option<SecondaryTarget>,
    /// Frequency range of the error [Hz].
    pub f_lo: f64,
    pub f_hi: f64,
    /// Fit level too; otherwise only the shape.
    pub seek_level: bool,
    /// Penalty when the impedance seen by the generator drops below this [Ω].
    pub min_z: Option<f64>,
    /// Penalty when the filter gain of any driver exceeds this [dB].
    pub max_gain: Option<f64>,
    /// Penalty when the main response varies more than this within the range [dB].
    pub linearity: Option<f64>,
    pub max_evals: usize,
    /// Round the result to this series.
    pub round: Option<Snap>,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            measure: Measure::ListeningWindow,
            level: 80.0,
            tilt: 0.0,
            hp: None,
            lp: None,
            target_file: None,
            file_offset: 0.0,
            secondary: None,
            f_lo: 200.0,
            f_hi: 8000.0,
            seek_level: false,
            min_z: None,
            max_gain: None,
            linearity: None,
            max_evals: 300,
            round: Some(Snap::E24),
        }
    }
}

impl Settings {
    /// Main target at `f` [dB].
    pub fn target_db(&self, f: f64) -> f64 {
        if let Some(r) = self.target_file.as_ref().filter(|r| r.len() >= 2) {
            let db = r.db();
            let i = r.freq.partition_point(|&x| x <= f).clamp(1, r.len() - 1);
            let t = ((f / r.freq[i - 1]).ln() / (r.freq[i] / r.freq[i - 1]).ln()).clamp(0.0, 1.0);
            return db[i - 1] + t * (db[i] - db[i - 1]) + self.file_offset;
        }
        self.level + self.tilt * (f / 1000.0).log2() + self.hp.map_or(0.0, |e| e.db(f, true)) + self.lp.map_or(0.0, |e| e.db(f, false))
    }

    /// Target of the second response at `f` [dB].
    pub fn secondary_target_db(&self, f: f64) -> f64 {
        self.secondary.map_or(0.0, |s| self.target_db(f) + s.level + s.tilt * (f / 1000.0).log2())
    }
}

/// A component to adjust: schematic index and optional limits (None = value/10 … value×10).
#[derive(Clone, Copy, Debug)]
pub struct Param {
    pub index: usize,
    pub min: Option<f64>,
    pub max: Option<f64>,
}

/// Result of a run: new values for the given components (same order) and the error before / after.
#[derive(Clone, Debug)]
pub struct Outcome {
    pub values: Vec<f64>,
    pub error_before: f64,
    pub error_after: f64,
    pub evals: usize,
}

pub struct Optimizer {
    sim: Simulation,
    project: Project,
    settings: Settings,
    freq: Vec<f64>,
    target: Vec<f64>,
    target2: Vec<f64>,
    /// gains[direction][source][frequency]; the circuit driver feeding each source is in `feeder`
    gains: Vec<Vec<Vec<Complex64>>>,
    feeder: Vec<usize>,
    /// direction indices of the reference axis and of the listening window
    ref_dir: usize,
    lw_dirs: Vec<usize>,
    /// ring directions: index, power weight, early-reflection bounce
    rings: Vec<(usize, f64, Option<usize>)>,
    /// part index, parameter name, bounds
    params: Vec<(usize, &'static str, f64, f64)>,
    evals: usize,
}

fn param_name(k: PartKind) -> Option<&'static str> {
    match k {
        PartKind::Resistor => Some("R"),
        PartKind::Capacitor => Some("C"),
        PartKind::Inductor => Some("L"),
        _ => None,
    }
}

impl Optimizer {
    /// `parts` = the schematic components to adjust (their indices are the project's part indices too).
    pub fn new(speaker: &Speaker, parts: &[Param], settings: &Settings) -> Result<Self, String> {
        let (sim, project) = speaker.evaluate_project()?;
        let freq: Vec<f64> = sim.freq.iter().copied().filter(|&f| f >= settings.f_lo && f <= settings.f_hi).collect();
        if freq.len() < 3 {
            return Err("frequency range of the optimizer is too narrow".into());
        }
        let first = sim.freq.iter().position(|&f| f >= settings.f_lo).unwrap();
        // every direction is computed once, whichever sums use it
        let mut dirs: Vec<(f64, f64)> = vec![];
        let mut index: HashMap<(i32, i32), usize> = HashMap::new();
        let mut dir = |h: f64, v: f64| *index.entry((h as i32, v as i32)).or_insert_with(|| {
            dirs.push((h, v));
            dirs.len() - 1
        });
        let ref_dir = dir(0.0, 0.0);
        let lw_dirs: Vec<usize> = if settings.measure == Measure::ListeningWindow || settings.secondary.is_some_and(|s| s.kind == Secondary::InRoom) { LW_DIRS.iter().map(|&(h, v)| dir(h, v)).collect() } else { vec![ref_dir] };
        let rings: Vec<(usize, f64, Option<usize>)> = if settings.secondary.is_some() { sim.rings().into_iter().map(|(h, v, w, b)| (dir(h, v), w, b)).collect() } else { vec![] };
        let gains = dirs.iter().map(|&(h, v)| (0..sim.source_count()).map(|j| sim.source_gain(j, h, v)[first..first + freq.len()].to_vec()).collect()).collect();
        let feeder = (0..sim.source_count()).map(|j| sim.source_feeder(j)).collect();
        let mut params = vec![];
        for p in parts {
            let part = project.parts.get(p.index).ok_or("component not in the circuit")?;
            let name = param_name(part.kind).ok_or("only R, C and L can be optimized")?;
            let v = part.param(name).filter(|&v| v > 0.0).ok_or("component value must be positive")?;
            let (lo, hi) = (p.min.filter(|&m| m > 0.0).unwrap_or(v / 10.0), p.max.filter(|&m| m > 0.0).unwrap_or(v * 10.0));
            if lo >= hi {
                return Err("a component has its minimum above its maximum".into());
            }
            params.push((p.index, name, lo, hi));
        }
        if params.is_empty() {
            return Err("no component selected for optimizing".into());
        }
        let target = freq.iter().map(|&f| settings.target_db(f)).collect();
        let target2 = freq.iter().map(|&f| settings.secondary_target_db(f)).collect();
        Ok(Optimizer { sim, project, settings: settings.clone(), freq, target, target2, gains, feeder, ref_dir, lw_dirs, rings, params, evals: 0 })
    }

    /// Error for component values `v` (same order as the components given to `new`).
    pub fn error(&mut self, v: &[f64]) -> f64 {
        self.evals += 1;
        for (&(k, name, _, _), &x) in self.params.iter().zip(v) {
            self.project.parts[k].params.insert(name.to_string(), x);
        }
        let Ok(sol) = self.sim.resolve(&self.project, &self.freq) else { return 1e9 };
        let n = self.freq.len();
        let st = self.settings.secondary;
        let w2 = st.map_or(0.0, |s| s.weight.clamp(0.0, 1.0));
        let (mut db1, mut d1, mut d2) = (vec![0.0; n], vec![0.0; n], vec![0.0; n]);
        let mut ms = vec![0.0; self.gains.len()];
        for i in 0..n {
            for (m, dir) in ms.iter_mut().zip(&self.gains) {
                *m = dir.iter().zip(&self.feeder).map(|(g, &j)| g[i] * sol.driver_v[j][i]).sum::<Complex64>().norm_sqr();
            }
            let lw = self.lw_dirs.iter().map(|&d| ms[d]).sum::<f64>() / self.lw_dirs.len() as f64;
            let main = if self.settings.measure == Measure::RefAxis { ms[self.ref_dir] } else { lw };
            db1[i] = 10.0 * main.max(1e-30).log10();
            d1[i] = db1[i] - self.target[i];
            if let Some(s) = st {
                let (sp, er) = ring_sums(self.rings.iter().map(|&(d, w, b)| (w, b, ms[d])));
                let sec = match s.kind {
                    Secondary::Power => sp,
                    Secondary::InRoom => in_room_ms(lw, er, sp),
                };
                d2[i] = 10.0 * sec.max(1e-30).log10() - self.target2[i];
            }
        }
        if !self.settings.seek_level {
            let shift = (1.0 - w2) * d1.iter().sum::<f64>() / n as f64 + w2 * d2.iter().sum::<f64>() / n as f64;
            d1.iter_mut().for_each(|d| *d -= shift);
            d2.iter_mut().for_each(|d| *d -= shift);
        }
        let mean_sq = |d: &[f64]| d.iter().map(|x| x * x).sum::<f64>() / n as f64;
        let mut err = (1.0 - w2) * mean_sq(&d1) + w2 * mean_sq(&d2);
        // penalties: 1 unit (Ω, dB) over the limit counts as 4 dB² (2 dB rms) over the whole range
        if let Some(zmin) = self.settings.min_z {
            err += 4.0 * sol.generator_z.iter().map(|z| (zmin - z.norm()).max(0.0).powi(2)).sum::<f64>() / n as f64;
        }
        if let Some(limit) = self.settings.max_gain {
            let worst = sol.driver_v.iter().map(|v| v.iter().map(|x| 20.0 * x.norm().max(1e-30).log10()).fold(f64::MIN, f64::max)).fold(f64::MIN, f64::max);
            err += 4.0 * (worst - limit).max(0.0).powi(2);
        }
        if let Some(limit) = self.settings.linearity {
            let dev = db1.iter().cloned().fold(f64::MIN, f64::max) - db1.iter().cloned().fold(f64::MAX, f64::min);
            err += 4.0 * (dev - limit).max(0.0).powi(2);
        }
        err
    }

    /// Runs the solver from the current values.
    pub fn run(&mut self) -> Outcome {
        let x0: Vec<f64> = self.params.iter().map(|&(k, name, ..)| self.project.parts[k].param(name).unwrap()).collect();
        let lo: Vec<f64> = self.params.iter().map(|p| p.2.ln()).collect();
        let hi: Vec<f64> = self.params.iter().map(|p| p.3.ln()).collect();
        // a start value outside its limits is first brought inside
        let x0: Vec<f64> = x0.iter().enumerate().map(|(i, v)| v.ln().clamp(lo[i], hi[i]).exp()).collect();
        let error_before = self.error(&x0);
        let mut best = x0.iter().map(|v| v.ln()).collect::<Vec<_>>();
        let mut best_err = error_before;
        let budget = self.settings.max_evals.max(10);
        // restarts: the simplex collapses on a plateau long before a minimum is certain
        while self.evals < budget {
            let before = best_err;
            let left = budget - self.evals;
            let (x, e) = nelder_mead(&mut |x| self.error(&x.iter().map(|v| v.exp()).collect::<Vec<_>>()), &best, &lo, &hi, left);
            if e < best_err {
                (best, best_err) = (x, e);
            }
            if before - best_err < 1e-6 * before.max(1e-9) {
                break;
            }
        }
        let mut values: Vec<f64> = best.iter().map(|v| v.exp()).collect();
        let mut error_after = best_err;
        if let Some(snap) = self.settings.round.filter(|&s| s != Snap::Percent5) {
            values.iter_mut().zip(self.params.iter()).for_each(|(v, p)| *v = nearest(*v, snap).clamp(p.2, p.3));
            error_after = self.error(&values);
        }
        Outcome { values, error_before, error_after, evals: self.evals }
    }
}

/// Value of the series closest (in ratio) to `v`.
pub fn nearest(v: f64, snap: Snap) -> f64 {
    let below = step(v * (1.0 + 1e-5), false, snap);
    let above = step(v * (1.0 - 1e-5), true, snap);
    if v / below <= above / v { below } else { above }
}

/// Nelder–Mead minimization inside the box [lo, hi]; returns the best point, its value and uses at most `max_evals` evaluations.
fn nelder_mead(f: &mut dyn FnMut(&[f64]) -> f64, x0: &[f64], lo: &[f64], hi: &[f64], max_evals: usize) -> (Vec<f64>, f64) {
    let n = x0.len();
    let clamp = |x: &mut Vec<f64>| x.iter_mut().enumerate().for_each(|(i, v)| *v = v.clamp(lo[i], hi[i]));
    let mut evals = 0;
    let mut eval = |x: &Vec<f64>, evals: &mut usize| {
        *evals += 1;
        f(x)
    };
    let mut simplex: Vec<(Vec<f64>, f64)> = vec![];
    let mut p = x0.to_vec();
    clamp(&mut p);
    simplex.push((p.clone(), eval(&p, &mut evals)));
    for i in 0..n {
        let mut q = x0.to_vec();
        q[i] += if q[i] + 0.3 <= hi[i] { 0.3 } else { -0.3 };
        clamp(&mut q);
        let e = eval(&q, &mut evals);
        simplex.push((q, e));
    }
    while evals + 2 <= max_evals {
        simplex.sort_by(|a, b| a.1.total_cmp(&b.1));
        let (best, worst) = (simplex[0].1, simplex[n].1);
        let size = simplex.iter().skip(1).map(|s| s.0.iter().zip(&simplex[0].0).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max)).fold(0.0, f64::max);
        if worst - best <= 1e-12 || size < 1e-5 {
            break;
        }
        let centroid: Vec<f64> = (0..n).map(|i| simplex[..n].iter().map(|s| s.0[i]).sum::<f64>() / n as f64).collect();
        let along = |t: f64| {
            let mut x: Vec<f64> = (0..n).map(|i| centroid[i] + t * (simplex[n].0[i] - centroid[i])).collect();
            clamp(&mut x);
            x
        };
        let xr = along(-1.0);
        let er = eval(&xr, &mut evals);
        if er < simplex[0].1 {
            let xe = along(-2.0);
            let ee = eval(&xe, &mut evals);
            simplex[n] = if ee < er { (xe, ee) } else { (xr, er) };
        } else if er < simplex[n - 1].1 {
            simplex[n] = (xr, er);
        } else {
            let xc = if er < worst { along(-0.5) } else { along(0.5) };
            let ec = eval(&xc, &mut evals);
            if ec < worst.min(er) {
                simplex[n] = (xc, ec);
            } else {
                let b = simplex[0].0.clone();
                for s in simplex.iter_mut().skip(1) {
                    for i in 0..n {
                        s.0[i] = b[i] + 0.5 * (s.0[i] - b[i]);
                    }
                    s.1 = eval(&s.0.clone(), &mut evals);
                }
            }
        }
    }
    simplex.sort_by(|a, b| a.1.total_cmp(&b.1));
    simplex.swap_remove(0)
}
