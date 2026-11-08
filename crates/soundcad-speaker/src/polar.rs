//! Polar responses measured in the finished cabinet: one file per angle and plane (angle and plane are read
//! from the file name by `soundcad_core::angle`). Used as they are: the front does not change them.

use soundcad_core::{Plane, Response};
use soundcad_xover::DriverResponses;

/// Builds the horizontal and vertical sets from measurement files (angle and plane set on each response).
/// Both sets need a 0° response; when only one plane has it, it is copied to the other.
pub fn from_measurements(files: Vec<Response>, impedance: Option<Response>) -> Result<DriverResponses, String> {
    let (mut hor, mut ver): (Vec<(f64, Response)>, Vec<(f64, Response)>) = (vec![], vec![]);
    for r in files {
        let angle = r.angle.ok_or_else(|| format!("{}: no angle in the file name (e.g. “name hor -30.txt”)", r.name))?;
        let set = if r.plane == Plane::Ver { &mut ver } else { &mut hor };
        // the same angle twice: the later file wins
        set.retain(|(a, _)| *a != angle);
        set.push((angle, r));
    }
    let zero = |set: &[(f64, Response)]| set.iter().find(|(a, _)| *a == 0.0).map(|(_, r)| r.clone());
    let (h0, v0) = (zero(&hor), zero(&ver));
    match (h0, v0) {
        (None, None) => return Err("a 0° (on-axis) response is needed".into()),
        (Some(h), None) => ver.push((0.0, h)),
        (None, Some(v)) => hor.push((0.0, v)),
        _ => {}
    }
    Ok(DriverResponses::new(hor, ver, impedance))
}

/// Name for a unit made of these files: their common start without separators, if there is one.
pub fn common_name(names: &[String]) -> Option<String> {
    let first = names.first()?;
    let n = (0..first.len()).take_while(|&i| first.is_char_boundary(i + 1) && names.iter().all(|s| s.starts_with(&first[..=i]))).count();
    let base = first[..n].trim_end_matches(|c: char| !c.is_alphanumeric() && c != ')').trim_end_matches("hor").trim_end_matches("ver").trim_end_matches(|c: char| !c.is_alphanumeric());
    (base.len() >= 2).then(|| base.to_string())
}

/// "hor −180…180 (37) · ver −90…90 (19)" style summary of a set.
pub fn summary(r: &DriverResponses) -> String {
    let range = |s: &[(f64, Response)]| match (s.first(), s.last()) {
        (Some(a), Some(b)) => format!("{:.0}…{:.0}° ({})", a.0, b.0, s.len()),
        _ => "none".into(),
    };
    format!("horizontal {} · vertical {}", range(&r.hor), range(&r.ver))
}
