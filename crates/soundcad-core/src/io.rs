//! Reading of txt / frd / zma response files.
//!
//! Parsing rules: tab, space or semicolon delimited; period is the decimal separator; comma is
//! the decimal separator when the data line contains no period; comma is a field
//! separator when the line contains both periods and commas. Lines that do not
//! start with two numbers (headers, comments) are skipped. Missing phase is 0.

use crate::angle::{parse_angle, AngleParsing};
use crate::response::{db_to_lin, Response};
use num_complex::Complex64;
use std::fmt;
use std::path::Path;

/// How the magnitude column is coded in a file.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MagUnit {
    /// dB (SPL, dBV, transfer functions). Linear input magnitude off.
    Db,
    /// Linear (impedance in ohm, pressure in Pa, volts). "Linear input mag" checked.
    Linear,
}

#[derive(Debug)]
pub enum Error {
    Io(std::io::Error),
    NoData(String),
    Unsorted(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(e) => write!(f, "{e}"),
            Error::NoData(n) => write!(f, "{n}: no numeric data lines"),
            Error::Unsorted(n) => write!(f, "{n}: frequencies are not strictly increasing"),
        }
    }
}

impl std::error::Error for Error {}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Error::Io(e)
    }
}

pub fn parse_line(line: &str) -> Option<Vec<f64>> {
    let line = line.trim();
    let has_period = line.contains('.');
    let fields: Vec<String> = if has_period {
        line.split(|c: char| c == '\t' || c == ' ' || c == ';' || c == ',')
            .filter(|s| !s.is_empty())
            .map(str::to_owned)
            .collect()
    } else {
        line.split(|c: char| c == '\t' || c == ' ' || c == ';')
            .filter(|s| !s.is_empty())
            .map(|s| s.replace(',', "."))
            .collect()
    };
    let mut nums = Vec::with_capacity(3);
    for f in fields.iter().take(3) {
        match f.parse::<f64>() {
            Ok(v) if v.is_finite() => nums.push(v),
            _ => break,
        }
    }
    (nums.len() >= 2).then_some(nums)
}

/// Parse response text. `name` is used for error messages and the response name.
pub fn parse_response(name: &str, text: &str, unit: MagUnit) -> Result<Response, Error> {
    let mut freq = Vec::new();
    let mut values = Vec::new();
    for line in text.lines() {
        let Some(n) = parse_line(line) else { continue };
        if n[0] <= 0.0 {
            continue;
        }
        let ph = n.get(2).copied().unwrap_or(0.0).to_radians();
        let mag = match unit {
            MagUnit::Db => db_to_lin(n[1]),
            MagUnit::Linear => n[1],
        };
        freq.push(n[0]);
        values.push(Complex64::from_polar(mag, ph));
    }
    if freq.is_empty() {
        return Err(Error::NoData(name.to_owned()));
    }
    if freq.windows(2).any(|w| w[1] <= w[0]) {
        return Err(Error::Unsorted(name.to_owned()));
    }
    Ok(Response::new(name, freq, values))
}


/// Read a response file; plane and angle are parsed from the file name.
pub fn read_response(path: &Path, unit: MagUnit, angles: &AngleParsing) -> Result<Response, Error> {
    let bytes = std::fs::read(path)?;
    let text = String::from_utf8_lossy(&bytes);
    let stem = path.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
    let mut r = parse_response(&stem, text.trim_start_matches('\u{feff}'), unit)?;
    if let Some((plane, angle)) = parse_angle(&stem, angles) {
        r.plane = plane;
        r.angle = Some(angle);
    }
    Ok(r)
}
