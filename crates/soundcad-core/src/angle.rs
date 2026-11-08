//! Angle and plane of a measurement from its file name (generic 2D mode).
//!
//! Horizontal plane is selected unless the vertical keyword is found. The angle
//! is the first (search from beginning) or last (search from end) number token
//! in the file name, divided by the angle multiplier. A number token starts at
//! the beginning of the name, after a non-alphanumeric character, or right after
//! "deg" (ARTA style `name_deg+110`). This covers ARTA and CLIO 2D naming.

use crate::response::Plane;

#[derive(Clone, Debug, PartialEq)]
pub struct AngleParsing {
    pub keyword_hor: String,
    pub keyword_ver: String,
    pub from_end: bool,
    pub multiplier: f64,
}

impl Default for AngleParsing {
    fn default() -> Self {
        Self { keyword_hor: "hor".into(), keyword_ver: "ver".into(), from_end: true, multiplier: 1.0 }
    }
}

pub fn parse_angle(stem: &str, opt: &AngleParsing) -> Option<(Plane, f64)> {
    let lower = stem.to_lowercase();
    let ver = opt.keyword_ver.to_lowercase();
    // Keyword must be a whole token: "drivername" does not contain plane "ver".
    let plane = if !ver.is_empty() && lower.split(|c: char| !c.is_ascii_alphanumeric()).any(|t| t == ver) {
        Plane::Ver
    } else {
        Plane::Hor
    };
    let tokens = number_tokens(&lower);
    let value = if opt.from_end { tokens.last() } else { tokens.first() }?;
    Some((plane, value / opt.multiplier))
}

fn number_tokens(s: &str) -> Vec<f64> {
    let b = s.as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    while i < b.len() {
        let boundary = i == 0 || !b[i - 1].is_ascii_alphanumeric() || s[..i].ends_with("deg");
        let signed = (b[i] == b'+' || b[i] == b'-') && i + 1 < b.len() && b[i + 1].is_ascii_digit();
        if boundary && (b[i].is_ascii_digit() || signed) {
            let start = i;
            i += 1;
            while i < b.len() && (b[i].is_ascii_digit() || b[i] == b'.') {
                i += 1;
            }
            // Token must end at a non-alphabetic char, otherwise it is part of a model name.
            if i == b.len() || !b[i].is_ascii_alphabetic() {
                if let Ok(v) = s[start..i].trim_end_matches('.').parse::<f64>() {
                    out.push(v);
                }
            }
        } else {
            i += 1;
        }
    }
    out
}
