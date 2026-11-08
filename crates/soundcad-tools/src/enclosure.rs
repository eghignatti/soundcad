//! Enclosure: driver T/S model in a box.
//!
//! Radiator types: infinite baffle, closed, bass reflex, passive radiator.
//!
//! * motional branch from the fundamental parameters Rms, Mms, Cms, Bl, Re
//!   (not fs/Qms/Qes), air ρ = 1.194 kg/m³, c = 344.0 m/s;
//! * total SPL to half space (2π) at 1 m from the cone volume velocity,
//!   voice coil inductance excluded ("Effect of voice coil inductance is not
//!   shown in SPL curve");
//! * impedance with the extended model: `Re + jωLeb + (Ke·√(jω) ∥ jωLe ∥ Rss)`;
//! * closed box: effective volume `Vb·(1 + 0.9·Qa^−0.9)`, absorption
//!   `Xc/Qa` in series with the box compliance, leakage `Ql·Xc` in parallel,
//!   `Xc` taken at the box resonance Fb.
//! * bass reflex: same lossy box plus port mass `Map = ρ·(L + EndCorr·D)/Sp`
//!   with loss `Rap = Xc/Qp` in parallel; Fb is the Helmholtz frequency of
//!   `Map` with the effective box compliance (driver not included). Radiated
//!   volume velocity = cone − port·e^(−jωL/c): port output delayed by its
//!   physical length. Leakage does not radiate.

use num_complex::Complex64;
use std::f64::consts::PI;
use soundcad_core::Response;
pub use soundcad_core::response::standard_grid;
pub use soundcad_core::{C_AIR, RHO};

const P_REF: f64 = 20e-6;

/// Driver record in driver database units.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Driver {
    pub manufacturer: String,
    pub model: String,
    /// DC resistance [Ω]
    pub re: f64,
    /// Bound inductance [mH]
    pub le: f64,
    /// Free inductance [mH]
    pub leb: f64,
    /// Semi-inductance [sH]
    pub ke: f64,
    /// Shunt resistance [Ω]; 0 = ignored (infinite)
    pub rss: f64,
    /// Mechanical resistance [Ns/m]
    pub rms: f64,
    /// Moving mass incl. air load [g]
    pub mms: f64,
    /// Compliance [mm/N]
    pub cms: f64,
    /// Effective cone area [cm²]
    pub sd: f64,
    /// Force factor [Tm]
    pub bl: f64,
}

/// Radiator type with its box parameters.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Radiator {
    InfiniteBaffle,
    /// Volume [l], absorption Qa, leakage Ql.
    Closed { vb: f64, qa: f64, ql: f64 },
    /// Closed box plus `n` round vents: diameter and length [cm], port loss Qp,
    /// end correction factor (effective length = len + end_corr·diam).
    BassReflex { vb: f64, qa: f64, ql: f64, diam: f64, len: f64, qp: f64, n: u32, end_corr: f64 },
    /// Closed box plus `pr.n` passive radiators.
    Passive { vb: f64, qa: f64, ql: f64, pr: PassiveRadiator },
}

/// Passive radiator parameters in database units.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PassiveRadiator {
    /// Equivalent volume of the suspension compliance [l].
    pub vas: f64,
    /// Effective area [cm²].
    pub sd: f64,
    pub qms: f64,
    /// Resonance frequency [Hz] (used for the loss resistance: Rap = 1/(2π·fs·Qms·Cap)); extra mass does not change it.
    pub fs: f64,
    /// Moving mass [g] and extra mass added per radiator [g].
    pub mms: f64,
    pub extra: f64,
    pub n: u32,
}

impl PassiveRadiator {
    /// fs from Vas, Sd and Mms, as stored in a driver database.
    pub fn natural_fs(&self) -> f64 {
        let cms = self.vas * 1e-3 / (RHO * C_AIR * C_AIR * (self.sd * 1e-4).powi(2));
        1.0 / (2.0 * PI * (self.mms * 1e-3 * cms).sqrt())
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Enclosure {
    pub driver: Driver,
    pub radiator: Radiator,
    /// Amplifier output voltage U [V].
    pub eg: f64,
    /// Internal resonances of the cabinet (see `cabinet`); they shape what the vent or passive radiator radiates.
    /// Empty = uniform pressure in the box.
    pub modes: Vec<crate::cabinet::Mode>,
}

/// Results of an enclosure simulation.
#[derive(Clone, Debug)]
pub struct Simulation {
    pub total_spl_2pi: Response,
    /// Cone alone and port alone (2π, 1 m); total = cone + port. Port `None` without a vent.
    pub cone_spl_2pi: Response,
    pub port_spl_2pi: Option<Response>,
    /// Passive radiator: the delay [s] of its output for the distance to the cone (radii sum);
    /// 0 otherwise. A model that places the radiator itself removes it.
    pub pr_delay: f64,
    pub impedance: Response,
    /// Box resonance [Hz]; `None` for infinite baffle.
    pub fb: Option<f64>,
}

/// Diameter [mm] of a circle of area `cm2` [cm²] (Sd → Dd).
pub fn area_diameter(cm2: f64) -> f64 {
    (4.0 * cm2 / PI).sqrt() * 10.0
}

fn par(a: Complex64, b: Complex64) -> Complex64 {
    a * b / (a + b)
}

impl Driver {
    fn si(&self) -> (f64, f64, f64, f64, f64) {
        (self.rms, self.mms * 1e-3, self.cms * 1e-3, self.sd * 1e-4, self.bl)
    }

    /// Voice coil impedance without Re: jωLeb + (Ke√(jω) ∥ jωLe ∥ Rss).
    pub fn voice_coil(&self, s: Complex64) -> Complex64 {
        let mut branches = Vec::with_capacity(3);
        if self.ke > 0.0 {
            branches.push(self.ke * s.sqrt());
        }
        if self.le > 0.0 {
            branches.push(s * (self.le * 1e-3));
        }
        // Rss shunts the inductive branches only; alone it would short nothing.
        if self.rss > 0.0 && !branches.is_empty() {
            branches.push(Complex64::new(self.rss, 0.0));
        }
        let z = branches.into_iter().reduce(par).unwrap_or_default();
        z + s * (self.leb * 1e-3)
    }

    /// Drivers of a .vxd driver database (XML): fundamental parameters in database units.
    pub fn from_vxd(xml: &str) -> Vec<Driver> {
        let tag = |b: &str, t: &str| -> String {
            let open = format!("<{t}>");
            b.find(&open).and_then(|s| b[s + open.len()..].find('<').map(|e| b[s + open.len()..s + open.len() + e].trim().to_string())).unwrap_or_default()
        };
        let num = |b: &str, t: &str| tag(b, t).parse::<f64>().unwrap_or(0.0);
        xml.split("<DRIVER>")
            .skip(1)
            .map(|b| Driver {
                manufacturer: tag(b, "Manufacturer"),
                model: tag(b, "Model"),
                re: num(b, "Re"),
                le: num(b, "Le"),
                leb: num(b, "Leb"),
                ke: num(b, "Ke"),
                rss: num(b, "Rss"),
                rms: num(b, "Rms"),
                mms: num(b, "Mms"),
                cms: num(b, "Cms"),
                sd: num(b, "Sd"),
                bl: num(b, "BL"),
            })
            .collect()
    }
}

/// Effective closed-box volume factor for absorption Qa.
pub fn absorption_volume_factor(qa: f64) -> f64 {
    1.0 + 0.9 * qa.powf(-0.9)
}

impl Enclosure {
    pub fn simulate(&self, freq: &[f64]) -> Simulation {
        let d = &self.driver;
        let (rms, mms, cms, sd, bl) = d.si();

        // Box in the acoustic domain: compliance Cab with absorption Rab = Xc/Qa in
        // series, leakage Ral = Ql·Xc and optional port (Rap + jωMap) in parallel.
        // Xc = 1/(ωb·Cab) at the box resonance Fb.
        struct Box {
            cab: f64,
            rab: f64,
            ral: f64,
            /// Map, Rap, physical length [m] (port output is delayed by len/c).
            port: Option<(f64, f64, f64)>,
            /// Passive radiators in parallel: acoustic mass, resistance, compliance (all radiators together) and the
            /// acoustic path [m] to the cone: radius of the radiator + radius of the driver.
            pr: Option<(f64, f64, f64, f64)>,
        }
        let resonance = |mass: f64, compliance: f64| 1.0 / (2.0 * PI * (mass * compliance).sqrt());
        let (bx, fb) = match self.radiator {
            Radiator::InfiniteBaffle => (None, None),
            Radiator::Closed { vb, qa, ql } | Radiator::BassReflex { vb, qa, ql, .. } | Radiator::Passive { vb, qa, ql, .. } => {
                let cab = vb * 1e-3 * absorption_volume_factor(qa) / (RHO * C_AIR * C_AIR);
                // Fb: driver on the box (closed), vent or radiators on the box; port as (Map, Qp, length)
                let (fb, port, pr) = match self.radiator {
                    Radiator::BassReflex { diam, len, qp, n, end_corr, .. } => {
                        let sp = n as f64 * PI * (diam * 0.5e-2).powi(2);
                        let map = RHO * (len + end_corr * diam) * 1e-2 / sp;
                        (resonance(map, cab), Some((map, qp, len * 1e-2)), None)
                    }
                    Radiator::Passive { pr, .. } => {
                        let n = pr.n as f64;
                        let (sdp, mms) = (pr.sd * 1e-4, pr.mms * 1e-3);
                        // acoustic values of all n radiators in parallel
                        let map = (mms + pr.extra * 1e-3) / n / (sdp * sdp);
                        let cap = n * pr.vas * 1e-3 / (RHO * C_AIR * C_AIR);
                        let rap = 1.0 / (2.0 * PI * pr.fs * pr.qms * cap);
                        let fb = resonance(map, 1.0 / (1.0 / cab + 1.0 / cap));
                        (fb, None, Some((map, rap, cap, (pr.sd.sqrt() + d.sd.sqrt()) / PI.sqrt() * 1e-2)))
                    }
                    _ => (resonance(mms, 1.0 / (1.0 / cms + sd * sd / cab)), None, None),
                };
                let xc = 1.0 / (2.0 * PI * fb * cab);
                (Some(Box { cab, rab: xc / qa, ral: ql * xc, port: port.map(|(map, qp, len)| (map, xc / qp, len)), pr }), Some(fb))
            }
        };

        let bx_pr_delay = bx.as_ref().and_then(|b| b.pr).map_or(0.0, |p| p.3 / C_AIR);
        let mut spl = Vec::with_capacity(freq.len());
        let (mut cone, mut vent) = (Vec::with_capacity(freq.len()), Vec::with_capacity(freq.len()));
        let mut imp = Vec::with_capacity(freq.len());
        for &f in freq {
            let s = Complex64::new(0.0, 2.0 * PI * f);
            let mut zm = rms + s * mms + 1.0 / (s * cms);
            // Fraction of cone volume velocity leaving through the port.
            let mut port_share = Complex64::default();
            if let Some(b) = &bx {
                let mut y = 1.0 / (b.rab + 1.0 / (s * b.cab)) + 1.0 / b.ral;
                let port = match (b.port, b.pr) {
                    (Some((map, rap, len)), _) => Some((rap + s * map, len)),
                    (None, Some((map, rap, cap, path))) => Some((rap + s * map + 1.0 / (s * cap), path)),
                    _ => None,
                };
                if let Some((zp, _)) = port {
                    y += 1.0 / zp;
                }
                zm += sd * sd / y;
                if let Some((zp, len)) = port {
                    port_share = (-s * (len / C_AIR)).exp() / (y * zp) * crate::cabinet::vent_factor(&self.modes, f);
                }
            }
            let u = bl * self.eg / d.re / (zm + bl * bl / d.re);
            let pc = RHO * sd * s * u / (2.0 * PI) / P_REF;
            cone.push(pc);
            vent.push(-pc * port_share);
            spl.push(pc * (1.0 - port_share));
            imp.push(d.re + d.voice_coil(s) + bl * bl / zm);
        }
        let name = format!("{} {}", d.manufacturer, d.model);
        Simulation {
            total_spl_2pi: Response::new(format!("{name} TotalSPL2pi"), freq.to_vec(), spl),
            cone_spl_2pi: Response::new(format!("{name} ConeSPL2pi"), freq.to_vec(), cone),
            pr_delay: bx_pr_delay,
            port_spl_2pi: matches!(self.radiator, Radiator::BassReflex { .. } | Radiator::Passive { .. }).then(|| Response::new(format!("{name} PortSPL2pi"), freq.to_vec(), vent)),
            impedance: Response::new(format!("{name} Impedance"), freq.to_vec(), imp),
            fb,
        }
    }
}
