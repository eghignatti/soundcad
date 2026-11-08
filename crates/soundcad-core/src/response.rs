use num_complex::Complex64;
use std::f64::consts::PI;

/// Measurement plane of an off-axis response.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum Plane {
    #[default]
    Hor,
    Ver,
}

/// Complex response on a frequency grid.
///
/// `values` are linear complex numbers: for SPL files `10^(dB/20)·e^{jφ}`,
/// for impedance files `|Z|·e^{jφ}`. Linear vs dB is a property of the file
/// (see [`crate::io::MagUnit`]), never of the math.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Response {
    pub name: String,
    pub freq: Vec<f64>,
    pub values: Vec<Complex64>,
    pub plane: Plane,
    /// Off-axis angle in degrees, if known.
    pub angle: Option<f64>,
}

impl Response {
    pub fn new(name: impl Into<String>, freq: Vec<f64>, values: Vec<Complex64>) -> Self {
        assert_eq!(freq.len(), values.len(), "freq and values length differ");
        Self { name: name.into(), freq, values, plane: Plane::Hor, angle: None }
    }

    pub fn from_db_deg(name: impl Into<String>, freq: Vec<f64>, db: &[f64], deg: &[f64]) -> Self {
        let values = db.iter().zip(deg).map(|(&m, &p)| from_db_deg(m, p)).collect();
        Self::new(name, freq, values)
    }

    pub fn len(&self) -> usize {
        self.freq.len()
    }

    pub fn is_empty(&self) -> bool {
        self.freq.is_empty()
    }

    pub fn db(&self) -> Vec<f64> {
        self.values.iter().map(|v| to_db(v.norm())).collect()
    }

    /// Phase in degrees wrapped to (-180, 180].
    pub fn phase_deg(&self) -> Vec<f64> {
        self.values.iter().map(|v| v.arg().to_degrees()).collect()
    }

    /// Same metadata, new values on the same grid.
    pub fn with_values(&self, name: impl Into<String>, values: Vec<Complex64>) -> Self {
        Self { name: name.into(), freq: self.freq.clone(), values, plane: self.plane, angle: self.angle }
    }

    pub fn map(&self, name: impl Into<String>, f: impl Fn(f64, Complex64) -> Complex64) -> Self {
        let values = self.freq.iter().zip(&self.values).map(|(&fr, &v)| f(fr, v)).collect();
        self.with_values(name, values)
    }

    /// Value at an arbitrary frequency: log-frequency interpolation of dB
    /// magnitude and unwrapped phase, clamped at the ends.
    pub fn sample_at(&self, f: f64, unwrapped: &[f64]) -> Complex64 {
        let n = self.len();
        if f <= self.freq[0] {
            return self.values[0];
        }
        if f >= self.freq[n - 1] {
            return self.values[n - 1];
        }
        let i = self.freq.partition_point(|&x| x <= f) - 1;
        let (f0, f1) = (self.freq[i], self.freq[i + 1]);
        let t = (f / f0).ln() / (f1 / f0).ln();
        let m0 = self.values[i].norm().max(1e-300).ln();
        let m1 = self.values[i + 1].norm().max(1e-300).ln();
        let mag = (m0 + t * (m1 - m0)).exp();
        let ph = unwrapped[i] + t * (unwrapped[i + 1] - unwrapped[i]);
        Complex64::from_polar(mag, ph)
    }

    /// Resample onto another grid. Returns self unchanged when grids match.
    pub fn resample(&self, freq: &[f64]) -> Response {
        if same_grid(&self.freq, freq) {
            return self.clone();
        }
        let un = unwrap(&self.values);
        let values = freq.iter().map(|&f| self.sample_at(f, &un)).collect();
        Response { freq: freq.to_vec(), values, ..self.clone() }
    }
}

/// Standard frequency grid: 48 points/octave from 5 Hz, 623 points (to ≈39.8 kHz).
pub fn standard_grid() -> Vec<f64> {
    grid_points(0..=622)
}

/// Points `i` of the standard grid, 5·2^(i/48) Hz (negative `i` below 5 Hz).
pub fn grid_points(i: std::ops::RangeInclusive<i32>) -> Vec<f64> {
    i.map(|i| 5.0 * 2f64.powf(i as f64 / 48.0)).collect()
}

pub fn same_grid(a: &[f64], b: &[f64]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(x, y)| ((x - y) / y).abs() < 1e-9)
}

/// Unwrapped phase in radians.
pub fn unwrap(values: &[Complex64]) -> Vec<f64> {
    let mut out = Vec::with_capacity(values.len());
    let mut offset = 0.0;
    let mut prev = 0.0;
    for (i, v) in values.iter().enumerate() {
        let p = v.arg();
        if i > 0 {
            let d = p - prev;
            if d > PI {
                offset -= 2.0 * PI;
            } else if d < -PI {
                offset += 2.0 * PI;
            }
        }
        prev = p;
        out.push(p + offset);
    }
    out
}

pub fn from_db_deg(db: f64, deg: f64) -> Complex64 {
    Complex64::from_polar(db_to_lin(db), deg.to_radians())
}

pub fn to_db(lin: f64) -> f64 {
    20.0 * lin.max(1e-300).log10()
}

pub fn db_to_lin(db: f64) -> f64 {
    10f64.powf(db / 20.0)
}
