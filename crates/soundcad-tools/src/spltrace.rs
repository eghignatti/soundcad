//! Graph Trace: frequency and impedance curves from a bitmap (datasheet graph).
//!
//! Two cursors per axis give the scale; a click on the curve starts
//! an automatic trace by colour, from left to right; the export has 48 points per octave, extended below
//! and above the traced range, with minimum phase, optionally smoothed 1/12 octave.
//!
//! Extensions: SPL continues with the slope of the lowest and of the highest octave of the trace. An impedance
//! with a resonance peak continues below the trace as the textbook resonance
//! `Re + (Zmax − Re) / (1 + jQ(f/fs − fs/f))`, with the Q that passes through the first traced point, never
//! below Re; above the trace it continues with the slope of the highest octave.

use num_complex::Complex64;
use soundcad_core::minphase::{fit_slope, minimum_phase};
use soundcad_core::response::{db_to_lin, grid_points};
use soundcad_core::smooth::{smooth, Smoothing};
use soundcad_core::Response;

/// RGBA image, 8 bits per channel, row by row from the top.
#[derive(Clone)]
pub struct Bitmap {
    pub width: usize,
    pub height: usize,
    pub rgba: Vec<u8>,
}

impl Bitmap {
    fn rgb(&self, x: usize, y: usize) -> [f32; 3] {
        let i = (y * self.width + x) * 4;
        let c = &self.rgba[i..i + 4];
        // transparent pixels count as a white page
        let a = c[3] as f32 / 255.0;
        [c[0] as f32 * a + 255.0 * (1.0 - a), c[1] as f32 * a + 255.0 * (1.0 - a), c[2] as f32 * a + 255.0 * (1.0 - a)]
    }
}

fn distance(a: [f32; 3], b: [f32; 3]) -> f32 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// Scale of one image axis: pixel `px0` shows value `v0`, pixel `px1` shows `v1`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Axis {
    pub px0: f64,
    pub px1: f64,
    pub v0: f64,
    pub v1: f64,
    pub log: bool,
}

impl Axis {
    pub fn is_valid(&self) -> bool {
        self.px0 != self.px1 && self.v0 != self.v1 && (!self.log || (self.v0 > 0.0 && self.v1 > 0.0))
    }

    pub fn value(&self, px: f64) -> f64 {
        let t = (px - self.px0) / (self.px1 - self.px0);
        if self.log { self.v0 * (self.v1 / self.v0).powf(t) } else { self.v0 + t * (self.v1 - self.v0) }
    }
}

/// The curve closest to a click, within `radius`: not the page and not a gray grid line. A line's pale edges and
/// the pixels where two curves touch have colours of their own: of the pixels around the closest one, the closest
/// whose colour is really that of a line, i.e. found all around. Returns its column, the centre of the curve in
/// that column, and its colour.
pub fn pick(bmp: &Bitmap, x: f64, y: f64, radius: i32, tolerance: f32) -> Option<(usize, f64, [f32; 3])> {
    let (cx, cy) = (x.round() as i32, y.round() as i32);
    let ink = |c: [f32; 3]| {
        let (max, min) = (c[0].max(c[1]).max(c[2]), c[0].min(c[1]).min(c[2]));
        !(max - min < 60.0 && max > 80.0) && 2.0 * (max - min) + (255.0 - max) > 60.0
    };
    // (squared distance from the click, x, y)
    let mut near: Vec<(i32, usize, usize)> = vec![];
    for py in (cy - radius).max(0)..=(cy + radius).min(bmp.height as i32 - 1) {
        for px in (cx - radius).max(0)..=(cx + radius).min(bmp.width as i32 - 1) {
            if ink(bmp.rgb(px as usize, py as usize)) {
                near.push(((px - cx).pow(2) + (py - cy).pow(2), px as usize, py as usize));
            }
        }
    }
    near.sort();
    let closest = (near.first()?.0 as f64).sqrt();
    near.retain(|n| (n.0 as f64).sqrt() <= closest + 2.5);
    // how many pixels of that colour around it: `follow` needs what a line has along its way
    let support = |&(_, px, py): &(i32, usize, usize)| {
        let c = bmp.rgb(px, py);
        let (x0, y0) = (px.saturating_sub(25), py.saturating_sub(25));
        (x0..(px + 26).min(bmp.width)).flat_map(|qx| (y0..(py + 26).min(bmp.height)).map(move |qy| (qx, qy))).filter(|&(qx, qy)| distance(bmp.rgb(qx, qy), c) < 40.0).count()
    };
    let counts: Vec<usize> = near.iter().map(support).collect();
    // the pale edge follows the line too, but as a colour it also fits a gray grid line: the purest colour first,
    // then the closest pixel
    let paleness = |&(_, px, py): &(i32, usize, usize)| {
        let c = bmp.rgb(px, py);
        (c[0].min(c[1]).min(c[2]) / 25.0) as u32
    };
    // a curve shows along the 51 columns looked at, even if partly covered; else the most frequent colour
    let k = (0..near.len())
        .filter(|&i| counts[i] >= 20)
        .min_by_key(|&i| (paleness(&near[i]), i))
        .or_else(|| (0..counts.len()).max_by_key(|&i| counts[i]))?;
    let (_, px, py) = near[k];
    let color = bmp.rgb(px, py);
    let run = runs(bmp, px, color, tolerance).into_iter().find(|r| r.top <= py as f64 && py as f64 <= r.bottom)?;
    Some((px, run.centre, color))
}

/// Vertical stretch of pixels of the curve colour in one column.
struct Run {
    top: f64,
    bottom: f64,
    centre: f64,
}

fn runs(bmp: &Bitmap, x: usize, color: [f32; 3], tolerance: f32) -> Vec<Run> {
    let mut out = vec![];
    let mut open: Option<(usize, f32, f32)> = None; // top, Σ weight, Σ weight·y
    for y in 0..=bmp.height {
        let d = if y < bmp.height { distance(bmp.rgb(x, y), color) } else { f32::MAX };
        if d < tolerance {
            let w = tolerance - d;
            let o = open.get_or_insert((y, 0.0, 0.0));
            o.1 += w;
            o.2 += w * y as f32;
        } else if let Some((top, w, wy)) = open.take() {
            out.push(Run { top: top as f64, bottom: (y - 1) as f64, centre: (wy / w) as f64 });
        }
    }
    out
}

/// Follows the curve of `color` from pixel (`x0`, `y0`) to the right up to column `x_end`: in each column the
/// stretch of that colour closest to the line through the last points. Stops where the curve is lost; a short
/// gap (another curve drawn over it) is bridged. Returns (x, y) in pixels.
pub fn follow(bmp: &Bitmap, x0: usize, y0: f64, color: [f32; 3], tolerance: f32, x_end: usize) -> Vec<(f64, f64)> {
    let max_gap = bmp.width / 12;
    let line = runs(bmp, x0, color, tolerance).into_iter().find(|r| r.top <= y0 && y0 <= r.bottom).map_or(2.0, |r| r.bottom - r.top + 1.0).min(8.0);
    let tall = bmp.height as f64 / 2.0; // a grid line of the same colour, not the curve
    let mut path = vec![(x0 as f64, y0)];
    let mut gap = 0;
    for x in x0 + 1..=x_end.min(bmp.width - 1) {
        let (expected, slope) = extrapolate(&path, x as f64);
        // the curve keeps moving during a gap, and it can be steep: the search must widen as fast
        let window = 6.0 + 1.5 * slope.abs() + (2.0 * gap as f64).min(40.0);
        let off = |r: &Run| if expected < r.top { r.top - expected } else if expected > r.bottom { expected - r.bottom } else { 0.0 };
        let found = runs(bmp, x, color, tolerance).into_iter().filter(|r| r.bottom - r.top < tall && off(r) <= window).min_by(|a, b| off(a).total_cmp(&off(b)));
        match found {
            Some(r) => {
                // longer than the line at this slope, or centred away from where it was going: two curves merged
                // at a crossing, stay on the expected line
                let merged = r.bottom - r.top + 1.0 > line + 1.5 * slope.abs() + 1.0 || (r.centre - expected).abs() > 1.0 + 0.5 * line + 0.5 * slope.abs();
                path.push((x as f64, if merged { expected.clamp(r.top, r.bottom) } else { r.centre }));
                gap = 0;
            }
            None => {
                gap += 1;
                if gap > max_gap {
                    break;
                }
            }
        }
    }
    path
}

/// A new stretch of trace replaces what the path had from its first column on; what lies between is interpolated
/// in the export (a click past a problem corrects the path).
pub fn join(path: &mut Vec<(f64, f64)>, new: Vec<(f64, f64)>) {
    if let Some(&(x, _)) = new.first() {
        path.retain(|p| p.0 < x);
        path.extend(new);
    }
}

/// Line through the last points: its value at `x` and its slope.
fn extrapolate(path: &[(f64, f64)], x: f64) -> (f64, f64) {
    let tail = &path[path.len().saturating_sub(6)..];
    let slope = fit_slope(tail.iter().copied());
    let (lx, ly) = tail[tail.len() - 1];
    (ly + slope * (x - lx), slope)
}

/// Traced pixels to (frequency, value), ascending in frequency.
pub fn to_points(path: &[(f64, f64)], x: &Axis, y: &Axis) -> Vec<(f64, f64)> {
    let mut p: Vec<(f64, f64)> = path.iter().map(|&(px, py)| (x.value(px), y.value(py))).filter(|q| q.0 > 0.0 && q.1.is_finite()).collect();
    p.sort_by(|a, b| a.0.total_cmp(&b.0));
    p
}

/// Grid of the export: the standard grid 5·2^(i/48), i = 0..622, reached from 1 Hz for the minimum phase.
const BELOW: i32 = 112;

fn grid() -> Vec<f64> {
    grid_points(-BELOW..=622)
}

/// Slope of `points` per octave between two frequencies.
fn slope_per_octave(points: &[(f64, f64)], f0: f64, f1: f64) -> f64 {
    fit_slope(points.iter().filter(|q| q.0 >= f0 && q.0 <= f1).map(|q| (q.0.log2(), q.1)))
}

/// Value of `points` at `f` inside their range, linear in log f.
fn inside(points: &[(f64, f64)], f: f64) -> f64 {
    let k = points.partition_point(|q| q.0 < f).clamp(1, points.len() - 1);
    let (a, b) = (points[k - 1], points[k]);
    if b.0 <= a.0 {
        return a.1;
    }
    a.1 + (b.1 - a.1) * (f / a.0).log2() / (b.0 / a.0).log2()
}

/// dB on the export grid → minimum-phase response on the standard grid, optionally smoothed.
fn finish(name: &str, freq: Vec<f64>, db: Vec<f64>, low_slope: f64, high_slope: f64, smoothed: bool) -> Response {
    let mag: Vec<f64> = db.iter().map(|&d| db_to_lin(d)).collect();
    let phase = minimum_phase(&freq, &mag, low_slope, high_slope);
    let k = BELOW as usize;
    let values = mag[k..].iter().zip(&phase[k..]).map(|(&m, &p)| Complex64::from_polar(m, p)).collect();
    let r = Response::new(name, freq[k..].to_vec(), values);
    if smoothed { smooth(&r, Smoothing::Oct(12)) } else { r }
}

/// Frequency response from traced (Hz, dB) points.
pub fn spl_response(name: &str, points: &[(f64, f64)], smoothed: bool) -> Result<Response, String> {
    if points.len() < 2 {
        return Err("the trace has fewer than 2 points".into());
    }
    let ((f0, v0), (f1, v1)) = (points[0], points[points.len() - 1]);
    let (low, high) = (slope_per_octave(points, f0, f0 * 2.0), slope_per_octave(points, f1 / 2.0, f1));
    let freq = grid();
    let db = freq
        .iter()
        .map(|&f| {
            if f < f0 {
                v0 + low * (f / f0).log2()
            } else if f > f1 {
                v1 + high * (f / f1).log2()
            } else {
                inside(points, f)
            }
        })
        .collect();
    Ok(finish(name, freq, db, low, high, smoothed))
}

/// Resonance `Re + (zmax − Re) / (1 + jQ(f/fs − fs/f))`.
fn resonance(f: f64, re: f64, zmax: f64, fs: f64, q: f64) -> f64 {
    (Complex64::new(re, 0.0) + Complex64::new(zmax - re, 0.0) / Complex64::new(1.0, q * (f / fs - fs / f))).norm()
}

/// The Q with which the resonance passes through |Z| = `z` at `f` (`f` below `fs`).
fn fit_q(f: f64, z: f64, re: f64, zmax: f64, fs: f64) -> f64 {
    let (mut a, mut b): (f64, f64) = (0.05, 50.0);
    for _ in 0..60 {
        let m = (a * b).sqrt();
        // a higher Q is a narrower peak: less impedance at `f`
        if resonance(f, re, zmax, fs, m) > z { a = m } else { b = m }
    }
    (a * b).sqrt()
}

/// Impedance response from traced (Hz, Ω) points; `re` = voice coil resistance.
pub fn impedance_response(name: &str, points: &[(f64, f64)], re: f64, smoothed: bool) -> Result<Response, String> {
    if points.len() < 2 {
        return Err("the trace has fewer than 2 points".into());
    }
    if re <= 0.0 {
        return Err("Re must be above zero".into());
    }
    let ((f0, z0), (f1, z1)) = (points[0], points[points.len() - 1]);
    // the resonance: highest point of the lower half of the trace, until it has clearly come down again
    let (mut fs, mut zmax) = (f0, z0);
    for &(f, z) in &points[..points.len() / 2] {
        if z > zmax {
            (fs, zmax) = (f, z);
        } else if z < zmax * 0.5 {
            break;
        }
    }
    let peak = fs > f0 && zmax > re;
    let q = if peak { fit_q(f0, z0, re, zmax, fs) } else { 0.0 };
    let db_of = |z: f64| 20.0 * z.max(1e-9).log10();
    let in_db: Vec<(f64, f64)> = points.iter().map(|&(f, z)| (f, db_of(z))).collect();
    let (low, high) = (slope_per_octave(&in_db, f0, f0 * 2.0), slope_per_octave(&in_db, f1 / 2.0, f1));
    let floor = db_of(re);
    let freq = grid();
    let db = freq
        .iter()
        .map(|&f| {
            if f < f0 {
                if peak { db_of(resonance(f, re, zmax, fs, q)) } else { (db_of(z0) + low * (f / f0).log2()).max(floor) }
            } else if f > f1 {
                db_of(z1) + high * (f / f1).log2()
            } else {
                inside(&in_db, f)
            }
        })
        .collect();
    Ok(finish(name, freq, db, 0.0, high, smoothed))
}
