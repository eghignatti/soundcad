//! Driver catalog: the user's own library of loudspeaker drivers (maker, model, type, T/S parameters,
//! measured or datasheet responses), stored as one JSON file and used to fill the Drivers tab.

use serde::{Deserialize, Serialize};
use std::f64::consts::PI;
use std::path::{Path, PathBuf};

use soundcad_core::Response;
use soundcad_tools::enclosure::{area_diameter, Driver, PassiveRadiator, Radiator, C_AIR, RHO};

use crate::Source;

/// Box of a driver that has none yet.
pub const DEFAULT_BOX: Radiator = Radiator::Closed { vb: 10.0, qa: 5.0, ql: 10.0 };

/// Effective diameter below which a driver of unknown type is taken for a tweeter (placement, catalog type), mm.
pub const TWEETER_DD: f64 = 40.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Kind {
    Woofer,
    Midrange,
    Tweeter,
    FullRange,
    /// Passive radiator (drone cone): only Mms, Cms, Rms, Sd are meaningful.
    Passive,
}

impl Kind {
    pub const ALL: [Kind; 5] = [Kind::Woofer, Kind::Midrange, Kind::Tweeter, Kind::FullRange, Kind::Passive];

    pub fn label(self) -> &'static str {
        match self {
            Kind::Woofer => "Woofer",
            Kind::Midrange => "Midrange",
            Kind::Tweeter => "Tweeter",
            Kind::FullRange => "Full range",
            Kind::Passive => "Passive radiator",
        }
    }
}

/// One catalog record.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Entry {
    pub kind: Kind,
    /// `driver.manufacturer` / `driver.model` are the maker and model; the rest are the T/S parameters.
    pub driver: Driver,
    /// Effective diameter Dd, mm (placement on the front).
    pub dd: f64,
    /// Overall (frame) diameter, mm: footprint on the front. 0 = unknown.
    #[serde(default)]
    pub outer_d: f64,
    /// Linear excursion, mm (one way).
    #[serde(default)]
    pub xmax: f64,
    #[serde(default)]
    pub notes: String,
    /// On-axis half-space response (measured or traced from the datasheet).
    #[serde(default)]
    pub response: Option<Response>,
    /// Off-axis half-space curves (angle in degrees, > 0, sorted). Stored only: the simulation uses `response`.
    #[serde(default)]
    pub off_axis: Vec<(f64, Response)>,
    #[serde(default)]
    pub impedance: Option<Response>,
}

/// Thiele/Small parameters derived from the fundamental ones.
#[derive(Clone, Copy, Debug)]
pub struct Derived {
    pub fs: f64,
    pub qms: f64,
    pub qes: f64,
    pub qts: f64,
    /// Equivalent volume, litres.
    pub vas: f64,
}

impl Entry {
    pub fn new(kind: Kind) -> Self {
        let mut driver = Driver { manufacturer: String::new(), model: String::new(), re: 5.0, le: 0.0, leb: 0.0, ke: 0.0, rss: 0.0, rms: 1.0, mms: 10.0, cms: 1.0, sd: 80.0, bl: 5.0 };
        if kind == Kind::Passive {
            (driver.re, driver.bl) = (0.0, 0.0);
        }
        Entry { kind, driver, dd: 100.0, outer_d: 0.0, xmax: 0.0, notes: String::new(), response: None, off_axis: vec![], impedance: None }
    }

    /// Stores an SPL curve at `angle` degrees (0 = `response`); true when it replaced one at the same angle.
    pub fn set_spl(&mut self, angle: f64, mut r: Response) -> bool {
        r.angle = Some(angle);
        if angle == 0.0 {
            return self.response.replace(r).is_some();
        }
        match self.off_axis.iter().position(|(a, _)| *a >= angle) {
            Some(i) if self.off_axis[i].0 == angle => {
                self.off_axis[i].1 = r;
                true
            }
            Some(i) => {
                self.off_axis.insert(i, (angle, r));
                false
            }
            None => {
                self.off_axis.push((angle, r));
                false
            }
        }
    }

    pub fn remove_spl(&mut self, angle: f64) {
        if angle == 0.0 {
            self.response = None;
        }
        self.off_axis.retain(|(a, _)| *a != angle);
    }

    /// Name of a unit made from this record.
    pub fn unit_name(&self) -> String {
        let t = self.title();
        if t.is_empty() { "Driver".to_string() } else { t }
    }

    /// As a project source: the response (box-corrected from the T/S unless a tweeter) or, without one, the T/S model.
    pub fn source(&self, radiator: Radiator) -> Source {
        match &self.response {
            Some(r) => Source::HalfSpace { response: r.clone(), impedance: self.impedance.clone(), box_correction: (self.kind != Kind::Tweeter).then(|| (self.driver.clone(), radiator)) },
            None => Source::Simulated { driver: self.driver.clone(), radiator },
        }
    }

    /// Copy kept in a project unit (`Unit::record`): everything but the response and impedance, which the unit's
    /// source already holds.
    pub fn record(&self) -> Entry {
        Entry { response: None, impedance: None, ..self.clone() }
    }

    /// All SPL curves by increasing angle.
    pub fn spl_curves(&self) -> Vec<(f64, &Response)> {
        self.response.iter().map(|r| (0.0, r)).chain(self.off_axis.iter().map(|(a, r)| (*a, r))).collect()
    }

    pub fn title(&self) -> String {
        format!("{} {}", self.driver.manufacturer, self.driver.model).trim().to_string()
    }

    /// Fs, Qms, Qes, Qts, Vas; `None` while Mms or Cms is not positive.
    pub fn derived(&self) -> Option<Derived> {
        let d = &self.driver;
        if d.mms <= 0.0 || d.cms <= 0.0 {
            return None;
        }
        let (mms, cms, sd) = (d.mms * 1e-3, d.cms * 1e-3, d.sd * 1e-4);
        let fs = 1.0 / (2.0 * PI * (mms * cms).sqrt());
        let w = 2.0 * PI * fs;
        let qms = if d.rms > 0.0 { w * mms / d.rms } else { f64::INFINITY };
        let qes = if d.bl > 0.0 { w * mms * d.re / (d.bl * d.bl) } else { f64::INFINITY };
        let qts = 1.0 / (1.0 / qms + 1.0 / qes);
        Some(Derived { fs, qms, qes, qts, vas: RHO * C_AIR * C_AIR * sd * sd * cms * 1e3 })
    }

    /// The record as a passive radiator of the enclosure model (Vas, Sd, Qms, Mms from Cms, Rms, Mms, Sd).
    pub fn passive(&self, n: u32, extra: f64) -> Option<PassiveRadiator> {
        let d = self.derived()?;
        d.qms.is_finite().then(|| PassiveRadiator { vas: d.vas, sd: self.driver.sd, qms: d.qms, fs: d.fs, mms: self.driver.mms, extra, n })
    }

    /// Same record for the catalog: same maker, model and type.
    pub fn same_record(&self, other: &Entry) -> bool {
        self.kind == other.kind && self.title() == other.title()
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Catalog {
    pub entries: Vec<Entry>,
}

impl Catalog {
    /// `data/catalog.json` beside the program; for a cargo build beside `target/`, so that `cargo clean` does not delete it.
    pub fn default_path(exe: &Path) -> PathBuf {
        let dir = exe.parent().unwrap_or(Path::new("."));
        let base = dir.ancestors().find(|a| a.file_name().is_some_and(|n| n == "target")).and_then(Path::parent).unwrap_or(dir);
        base.join("data").join("catalog.json")
    }

    pub fn load(path: &Path) -> Result<Catalog, String> {
        let s = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
        serde_json::from_str(&s).map_err(|e| e.to_string())
    }

    /// Writes through a temporary file: a crash during the write must not destroy the catalog.
    pub fn save(&self, path: &Path) -> Result<(), String> {
        let s = serde_json::to_string(self).map_err(|e| e.to_string())?;
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
        }
        let tmp = path.with_extension("json.tmp");
        std::fs::write(&tmp, s).map_err(|e| e.to_string())?;
        std::fs::rename(&tmp, path).map_err(|e| e.to_string())
    }

    /// Adds the drivers of a .vxd driver database as woofers; returns how many.
    pub fn import_vxd(&mut self, xml: &str) -> usize {
        let list = Driver::from_vxd(xml);
        let n = list.len();
        for d in list {
            self.entries.push(Entry { dd: area_diameter(d.sd), driver: d, ..Entry::new(Kind::Woofer) });
        }
        n
    }
}
