//! Crossover as a simple per-way ladder (quick filters, older project files): generator → series elements /
//! shunt branches → driver. `schematic::Schematic::from_ways` draws it for the editor and the solver.

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Element {
    /// Ω
    R(f64),
    /// µF
    C(f64),
    /// mH, DCR Ω
    L(f64, f64),
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Stage {
    /// Element in series toward the driver.
    Series(Element),
    /// Branch to ground (elements in series), e.g. a shunt C, or R+C Zobel.
    Shunt(Vec<Element>),
}

/// One way: the network from the generator to one driver.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Way {
    /// Driver model name (links to the speaker unit).
    pub driver: String,
    pub stages: Vec<Stage>,
    pub inverted: bool,
}

impl Way {
    pub fn low_pass_2(driver: &str, l_mh: f64, c_uf: f64) -> Self {
        Way { driver: driver.into(), stages: vec![Stage::Series(Element::L(l_mh, 0.0)), Stage::Shunt(vec![Element::C(c_uf)])], inverted: false }
    }
    pub fn high_pass_2(driver: &str, c_uf: f64, l_mh: f64) -> Self {
        Way { driver: driver.into(), stages: vec![Stage::Series(Element::C(c_uf)), Stage::Shunt(vec![Element::L(l_mh, 0.0)])], inverted: false }
    }
}

/// Quick filter shapes for a way, designed on a resistive load (the driver's nominal impedance).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    LowPass,
    HighPass,
    /// High-pass at `fc`, low-pass at `fc2`.
    BandPass,
}

/// 2nd order alignment.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Align {
    /// Q 0.707
    Butterworth,
    /// Q 0.5 (Linkwitz-Riley 2nd order)
    LinkwitzRiley,
}

/// Ladder stages of a textbook passive filter into resistive load `r` Ω: order 1…3, cut-off `fc` Hz
/// (1st and 3rd order Butterworth; 2nd order Butterworth or Linkwitz-Riley). Values in µF / mH.
pub fn filter_stages(shape: Shape, order: u32, align: Align, fc: f64, fc2: f64, r: f64) -> Vec<Stage> {
    let (l, c) = (|x: f64| Element::L(x * 1e3, 0.0), |x: f64| Element::C(x * 1e6));
    let q = if align == Align::Butterworth { std::f64::consts::FRAC_1_SQRT_2 } else { 0.5 };
    let lp = |f: f64| {
        let w = 2.0 * std::f64::consts::PI * f;
        match order {
            1 => vec![Stage::Series(l(r / w))],
            2 => vec![Stage::Series(l(r / (q * w))), Stage::Shunt(vec![c(q / (r * w))])],
            _ => vec![Stage::Series(l(1.5 * r / w)), Stage::Shunt(vec![c(4.0 / (3.0 * r * w))]), Stage::Series(l(0.5 * r / w))],
        }
    };
    let hp = |f: f64| {
        let w = 2.0 * std::f64::consts::PI * f;
        match order {
            1 => vec![Stage::Series(c(1.0 / (r * w)))],
            2 => vec![Stage::Series(c(q / (r * w))), Stage::Shunt(vec![l(r / (q * w))])],
            _ => vec![Stage::Series(c(2.0 / (3.0 * r * w))), Stage::Shunt(vec![l(0.75 * r / w)]), Stage::Series(c(2.0 / (r * w)))],
        }
    };
    match shape {
        Shape::LowPass => lp(fc),
        Shape::HighPass => hp(fc),
        Shape::BandPass => [hp(fc), lp(fc2)].concat(),
    }
}
