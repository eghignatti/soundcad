//! Integrated speaker model: one object holds the cabinet front, the drivers placed on it, their box
//! loading and the crossover; [`Speaker::evaluate`] runs the whole chain in memory
//! (no intermediate files):
//!
//! 1. half-space (2π) response of each unit: enclosure model from T/S + box, or a measured/datasheet
//!    infinite-baffle response, optionally box-corrected by the enclosure ratio box/infinite baffle;
//! 2. diffraction model on the cabinet front, mic on the unit's axis at the listening distance: full-space
//!    responses = half-space × cabinet impact / 2, horizontal and vertical planes every `angle_step`;
//!    units measured in the final cabinet skip 1–2;
//! 3. crossover: the circuit of `crossover`, driver positions taken from the front layout, bass-reflex
//!    ports as extra sources fed by their driver.

pub mod catalog;
pub mod network;
pub mod optimize;
pub mod polar;
pub mod schematic;

use serde::{Deserialize, Serialize};

use soundcad_core::{Plane, Response};
use soundcad_tools::diffraction::Baffle;
use soundcad_tools::enclosure::{self, Enclosure, Radiator};
use soundcad_xover::{DriverResponses, Project, Simulation};

/// Cabinet front (baffle) in mm, origin at the bottom-left corner.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Front {
    /// Outline corners (3…36), mm.
    pub corners: Vec<(f64, f64)>,
    /// Edge radius, mm.
    pub edge_radius: f64,
    /// Ideal (thin) edge instead of a real cabinet edge.
    pub ideal_edge: bool,
}

impl Front {
    pub fn rect(width: f64, height: f64, edge_radius: f64) -> Self {
        Front { corners: vec![(0.0, 0.0), (width, 0.0), (width, height), (0.0, height)], edge_radius, ideal_edge: false }
    }

    /// Width and height of the outline's bounding box, mm.
    pub fn size(&self) -> (f64, f64) {
        self.corners.iter().fold((0.0, 0.0), |(w, h), c| (w.max(c.0), h.max(c.1)))
    }
}

/// Where a unit's response comes from.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Source {
    /// Thiele/Small model in its box (enclosure model); baffle effects from the front.
    Simulated { driver: enclosure::Driver, radiator: Radiator },
    /// On-axis response in half space (infinite or IEC baffle, measured or traced from a datasheet),
    /// impedance optional; `box_correction` multiplies by enclosure(box)/enclosure(infinite baffle) and then
    /// takes the impedance from the box simulation.
    HalfSpace { response: Response, impedance: Option<Response>, box_correction: Option<(enclosure::Driver, Radiator)> },
    /// Polar responses measured in the final cabinet (horizontal and vertical sets with 0° in both):
    /// used as they are, the front does not change them.
    InCabinet { responses: DriverResponses },
}

/// A driver unit placed on the front.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Unit {
    /// Matches the `Model` of the crossover's Driver part.
    pub name: String,
    /// Centre on the front, mm; `z` = acoustic centre depth behind the front, mm.
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// Effective diameter, mm.
    pub dd: f64,
    /// Overall (frame) diameter, mm: drawing and placement only. 0 = unknown.
    #[serde(default)]
    pub outer_d: f64,
    pub source: Source,
    /// Bass-reflex port (bass-reflex box, T/S model or box correction): centre on the front and (equivalent)
    /// diameter, mm. Without it the vent radiates from the cone position.
    pub port: Option<(f64, f64, f64)>,
    /// Rectangular vent opening width × height, mm (drawing; the acoustics use the equivalent diameter).
    #[serde(default)]
    pub port_rect: Option<(f64, f64)>,
    /// Catalog record the driver came from (maker, type, Xmax, notes, off-axis curves), so the project carries the
    /// whole driver: whoever opens it can put the driver back into a catalog. The response and impedance are not
    /// repeated here: they are in `source`.
    #[serde(default)]
    pub record: Option<catalog::Entry>,
}

impl Unit {
    /// A unit at (`x`, `y`) on the front: no depth, frame size, vent or catalog record.
    pub fn new(name: impl Into<String>, (x, y): (f64, f64), dd: f64, source: Source) -> Self {
        Unit { name: name.into(), x, y, z: 0.0, dd, outer_d: 0.0, source, port: None, port_rect: None, record: None }
    }

    /// Diameter of the circle the unit takes on the front, mm.
    pub fn footprint(&self) -> f64 {
        self.dd.max(self.outer_d)
    }
}

/// `r` advanced by `delay` seconds (removes a delay: the radiator's position is modelled by the front).
fn advance(r: &Response, delay: f64) -> Response {
    if delay == 0.0 {
        return r.clone();
    }
    r.map(r.name.clone(), |f, v| v * num_complex::Complex64::from_polar(1.0, 2.0 * std::f64::consts::PI * f * delay))
}

#[derive(Clone, Debug)]
pub struct Speaker {
    pub front: Front,
    pub units: Vec<Unit>,
    /// Crossover circuit; Driver parts are linked to units by model name.
    pub crossover: Project,
    /// Point of the front the listening axis goes through, mm.
    pub axis: (f64, f64),
    /// Listening (reference) distance, mm.
    pub distance: f64,
    /// Polar step of the generated responses, degrees.
    pub angle_step: f64,
    /// Generator voltage of the T/S simulations, V (responses are 2.83 V-scaled in the crossover).
    pub eg: f64,
    /// Other crossover variants to show next to the main one: (name, circuit); see `compare_totals`.
    pub compare: Vec<(String, Project)>,
}

impl Speaker {
    pub fn new(front: Front, crossover: Project) -> Self {
        Speaker { front, units: vec![], crossover, axis: (0.0, 0.0), distance: 1000.0, angle_step: 10.0, eg: 2.83, compare: vec![] }
    }

    /// On-axis total of each variant in `compare`, on the grid of `sim` (the run of `evaluate_project`, whose
    /// project is `base`): same acoustics, the variant's circuit. Drivers are matched by model name; a driver
    /// missing from the variant is silent there, a variant that does not solve is left out.
    pub fn compare_totals(&self, sim: &Simulation, base: &Project) -> Vec<(String, Response)> {
        let models: Vec<&str> = sim.circuit.driver_parts.iter().map(|&k| base.parts[k].model.as_str()).collect();
        let gains: Vec<Vec<num_complex::Complex64>> = (0..sim.source_count()).map(|s| sim.source_gain(s, 0.0, 0.0)).collect();
        self.compare
            .iter()
            .filter_map(|(name, proj)| {
                let mut p = proj.clone();
                p.drivers = base.drivers.clone();
                let sol = sim.resolve(&p, &sim.freq).ok()?;
                let mut total = vec![num_complex::Complex64::default(); sim.freq.len()];
                for (s, g) in gains.iter().enumerate() {
                    let model = models[sim.source_feeder(s)];
                    let Some(j) = sol.driver_parts.iter().position(|&k| p.parts[k].model == model) else { continue };
                    for (i, t) in total.iter_mut().enumerate() {
                        *t += g[i] * sol.driver_v[j][i];
                    }
                }
                Some((name.clone(), Response::new(name.clone(), sim.freq.clone(), total)))
            })
            .collect()
    }

    /// Diffraction model of the front for one radiator (mic on its axis).
    fn baffle(&self, (x, y): (f64, f64), dd: f64) -> Baffle {
        // the diffraction model works in metres
        Baffle {
            corners: self.front.corners.iter().map(|c| (c.0 / 1000.0, c.1 / 1000.0)).collect(),
            drivers: vec![(x / 1000.0, y / 1000.0)],
            mic: (x / 1000.0, y / 1000.0),
            dd: dd / 1000.0,
            edge_radius: if self.front.ideal_edge { 0.0 } else { self.front.edge_radius / 1000.0 },
            axis_distance: self.distance / 1000.0,
            angle_hor: 0.0,
            angle_ver: 0.0,
            open: false,
            floor_y: None,
            wall_x: None,
            absorption_db: 0.0,
        }
    }

    /// Full-space polar set of a radiator from its half-space on-axis response.
    fn polars(&self, half: &Response, at: (f64, f64), dd: f64, impedance: Option<Response>) -> DriverResponses {
        let b = self.baffle(at, dd);
        let mut hor = vec![];
        let mut ver = vec![];
        for r in b.directivity(self.angle_step, true, true, &half.freq) {
            let a = r.angle.unwrap();
            let full = Response::new("", half.freq.clone(), half.values.iter().zip(&r.values).map(|(h, c)| h * c / 2.0).collect());
            if r.plane == Plane::Hor {
                if a == 0.0 {
                    ver.push((0.0, full.clone()));
                }
                hor.push((a, full));
            } else {
                ver.push((a, full));
            }
        }
        DriverResponses::new(hor, ver, impedance)
    }

    /// Runs the whole chain: half-space responses → front diffraction → crossover.
    pub fn evaluate(&self) -> Result<Simulation, String> {
        self.evaluate_project().map(|(sim, _)| sim)
    }

    /// As `evaluate`, also returning the solver project (drivers, positions and component values) it ran.
    pub fn evaluate_project(&self) -> Result<(Simulation, Project), String> {
        let freq = enclosure::standard_grid();
        let mut p = self.crossover.clone();
        let mut meas = vec![];
        let mut ports = vec![];
        p.drivers.clear();
        for u in &self.units {
            let (resp, port) = match &u.source {
                Source::InCabinet { responses } => {
                    // measurements come on their own frequency grids
                    let grid = |set: &[(f64, Response)]| set.iter().map(|(a, r)| (*a, r.resample(&freq))).collect::<Vec<_>>();
                    if responses.hor.is_empty() {
                        return Err(format!("{}: no polar responses loaded", u.name));
                    }
                    (DriverResponses { hor: grid(&responses.hor), ver: grid(&responses.ver), impedance: responses.impedance.clone() }, None)
                }
                source => {
                    let box_sim = |driver: &enclosure::Driver, radiator| Enclosure { driver: driver.clone(), radiator, eg: self.eg }.simulate(&freq);
                    // half-space cone and vent responses, the passive radiator's delay and the impedance
                    let (cone, vent, pr_delay, impedance) = match source {
                        Source::Simulated { driver, radiator } => {
                            let s = box_sim(driver, *radiator);
                            (s.cone_spl_2pi, s.port_spl_2pi, s.pr_delay, Some(s.impedance))
                        }
                        Source::HalfSpace { response, impedance, box_correction } => {
                            if response.len() < 2 {
                                return Err(format!("{}: no frequency response loaded", u.name));
                            }
                            let r = response.resample(&freq);
                            match box_correction {
                                // cone and vent = measurement × (box cone, box vent) / infinite baffle, impedance from the
                                // box simulation (a free-air measurement would show the wrong resonance)
                                Some((driver, radiator)) => {
                                    let (bx, ib) = (box_sim(driver, *radiator), box_sim(driver, Radiator::InfiniteBaffle));
                                    let scaled = |part: &Response| Response::new("", freq.clone(), r.values.iter().zip(part.values.iter().zip(&ib.total_spl_2pi.values)).map(|(v, (b, i))| v * b / i).collect());
                                    (scaled(&bx.cone_spl_2pi), bx.port_spl_2pi.as_ref().map(scaled), bx.pr_delay, Some(bx.impedance))
                                }
                                None => (r, None, 0.0, impedance.clone()),
                            }
                        }
                        Source::InCabinet { .. } => unreachable!(),
                    };
                    let port = vent.map(|ps| match u.port {
                        Some((px, py, pd)) => (self.polars(&advance(&ps, pr_delay), (px, py), pd, None), (px, py)),
                        // no port position given: the vent radiates from the cone
                        None => (self.polars(&ps, (u.x, u.y), u.dd, None), (u.x, u.y)),
                    });
                    (self.polars(&cone, (u.x, u.y), u.dd, impedance), port)
                }
            };
            p.drivers.push(soundcad_xover::project::DriverData {
                model: u.name.clone(),
                impedance_scale: 1.0,
                scale: 1.0,
                z: 8.0,
                ..Default::default()
            });
            meas.push(resp);
            ports.push(port);
        }
        // driver positions in the circuit come from the front layout
        for part in p.parts.iter_mut().filter(|x| x.kind == soundcad_xover::PartKind::Driver) {
            if let Some(u) = self.units.iter().find(|u| u.name == part.model) {
                part.params.insert("X".into(), u.x - self.axis.0);
                part.params.insert("Y".into(), u.y - self.axis.1);
                part.params.insert("Z".into(), u.z);
            }
        }
        let mut sim = soundcad_xover::simulate_with(&p, meas, self.distance)?;
        for (j, &k) in sim.circuit.driver_parts.clone().iter().enumerate() {
            let Some(ui) = self.units.iter().position(|u| u.name == p.parts[k].model) else { continue };
            if let Some((resp, (px, py))) = ports[ui].clone() {
                sim.add_coupled_source(j, resp, (px - self.axis.0, py - self.axis.1, 0.0));
            }
        }
        Ok((sim, p))
    }
}

/// Chart layout saved with the project: number of chart panels, the chart in each, smoothing (0 = none, else 1/N oct).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct View {
    pub panels: usize,
    pub charts: [usize; 6],
    pub smoothing: u32,
}

/// File type written into every project file.
pub const FORMAT: &str = "soundcad";
/// Version of the project file; a program reads every version up to its own.
pub const FORMAT_VERSION: u32 = 1;

/// Everything the user edits, as saved in a project file (.soundcad, JSON). The file is self-contained: measured
/// responses, impedances, polar sets, driver records, crossover variants, optimizer target and chart layout are all
/// inside it, so it can be sent to someone else as it is.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Design {
    pub front: Front,
    pub units: Vec<Unit>,
    /// Quick-filter templates, one per unit (linked by driver name); the schematic below is the crossover.
    pub ways: Vec<network::Way>,
    /// Crossover schematic as drawn (empty in older files: generated from `ways`).
    #[serde(default)]
    pub schematic: schematic::Schematic,
    /// Listening axis point on the front, mm.
    pub axis: (f64, f64),
    /// Listening distance, mm.
    pub distance: f64,
    /// Optimizer target and options (the components to adjust are flagged in the schematic).
    #[serde(default)]
    pub optimizer: optimize::Settings,
    #[serde(default)]
    pub variants: Variants,
    /// None: the program's default layout.
    #[serde(default)]
    pub view: Option<View>,
}

#[derive(Serialize)]
struct FileOut<'a> {
    format: &'a str,
    version: u32,
    #[serde(flatten)]
    design: &'a Design,
}

/// Header of a project file, read on its own: reading it together with the design through `flatten` would pass
/// every number through serde's buffer, which does not give back every f64 exactly.
#[derive(Deserialize)]
struct Header {
    format: Option<String>,
    #[serde(default)]
    version: u32,
}

/// Crossover variants (8 memories). The visible schematic belongs to variant
/// `active` and is stored into its slot whenever the variants are used or the project is saved.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Variants {
    pub slots: Vec<Option<schematic::Schematic>>,
    pub active: usize,
}

pub const VARIANT_COUNT: usize = 8;

impl Design {
    /// An empty design on `front`: no drivers, listening axis at `axis`, 1 m away.
    pub fn new(front: Front, axis: (f64, f64)) -> Self {
        Design { front, units: vec![], ways: vec![], schematic: Default::default(), axis, distance: 1000.0, optimizer: Default::default(), variants: Default::default(), view: None }
    }

    fn slots(&mut self) -> &mut Vec<Option<schematic::Schematic>> {
        self.variants.slots.resize(VARIANT_COUNT, None);
        &mut self.variants.slots
    }

    /// Stores the visible schematic into the active variant's slot.
    pub fn sync_variants(&mut self) {
        let (a, s) = (self.variants.active.min(VARIANT_COUNT - 1), self.schematic.clone());
        self.variants.active = a;
        self.slots()[a] = Some(s);
    }

    /// Copies the visible schematic to variant `k` (overwriting it) and makes `k` the visible variant.
    pub fn save_variant(&mut self, k: usize) {
        // the variant being left keeps what it showed
        self.sync_variants();
        self.variants.active = k.min(VARIANT_COUNT - 1);
        self.sync_variants();
    }

    /// Shows variant `k` if it exists; the one left is stored first.
    pub fn recall_variant(&mut self, k: usize) -> bool {
        self.sync_variants();
        let Some(s) = self.slots().get(k).cloned().flatten() else { return false };
        self.schematic = s;
        self.variants.active = k;
        true
    }

    /// Deletes variant `k` (not the visible one).
    pub fn delete_variant(&mut self, k: usize) -> bool {
        if k == self.variants.active || k >= VARIANT_COUNT {
            return false;
        }
        self.slots()[k] = None;
        true
    }

    /// `base`, or `base 2`, `base 3`… if another unit (not number `except`) already has that name.
    pub fn unique_name(&self, base: &str, except: Option<usize>) -> String {
        let taken = |n: &str| self.units.iter().enumerate().any(|(i, u)| Some(i) != except && u.name == n);
        let mut name = base.to_string();
        let mut n = 2;
        while taken(&name) {
            name = format!("{base} {n}");
            n += 1;
        }
        name
    }

    /// Follows a change of unit name (names are the link): ways, schematic and every stored variant.
    pub fn relink_driver(&mut self, old: &str, new: &str) {
        for w in self.ways.iter_mut().filter(|w| w.driver == old) {
            w.driver = new.to_string();
        }
        self.schematic.rename_driver(old, new);
        for s in self.variants.slots.iter_mut().flatten() {
            s.rename_driver(old, new);
        }
    }

    /// Removes a unit and its crossover branch, also from the stored variants.
    pub fn remove_unit(&mut self, i: usize) {
        let name = self.units.remove(i).name;
        self.ways.retain(|w| w.driver != name);
        self.schematic.remove_driver(&name);
        for s in self.variants.slots.iter_mut().flatten() {
            s.remove_driver(&name);
        }
    }

    /// Puts catalog driver `e` in place of unit `i`: position, depth, box, vent and crossover stay.
    pub fn replace_unit(&mut self, i: usize, e: &catalog::Entry) {
        let name = self.unique_name(&e.unit_name(), Some(i));
        let u = &mut self.units[i];
        let radiator = match &u.source {
            Source::Simulated { radiator, .. } | Source::HalfSpace { box_correction: Some((_, radiator)), .. } => *radiator,
            _ => catalog::DEFAULT_BOX,
        };
        let old = std::mem::replace(&mut u.name, name.clone());
        u.source = e.source(radiator);
        u.dd = e.dd;
        u.outer_d = e.outer_d;
        u.record = Some(e.record());
        self.relink_driver(&old, &name);
    }

    /// Centre for a new driver that touches nothing: on the centre line if possible, as close to it as possible
    /// otherwise (`centre_only`: only on it); tweeters from the top down, the others from the bottom up.
    // ponytail: bounding box of the front and vents as single circles; the editor only makes rectangles
    pub fn free_spot(&self, dd: f64, outer_d: f64, centre_only: bool) -> Option<(f64, f64)> {
        const GAP: f64 = 5.0;
        let d = dd.max(outer_d);
        let (w, h) = self.front.size();
        let inset = d / 2.0 + self.front.edge_radius.max(GAP);
        let (cols, rows) = ((w / 2.0 - inset) / GAP, (h - 2.0 * inset) / GAP);
        if cols < 0.0 || rows < 0.0 {
            return None;
        }
        let free = |x: f64, y: f64| {
            let clear = |cx: f64, cy: f64, cd: f64| (x - cx).hypot(y - cy) >= (d + cd) / 2.0 + GAP;
            self.units.iter().all(|u| {
                let port = u.port.map_or(true, |(px, py, pd)| clear(px, py, u.port_rect.map_or(pd, |(a, b)| a.hypot(b))));
                clear(u.x, u.y, u.footprint()) && port
            })
        };
        for c in 0..=if centre_only { 0 } else { cols as usize } {
            for x in [w / 2.0 - c as f64 * GAP, w / 2.0 + c as f64 * GAP] {
                for r in 0..=rows as usize {
                    let y = if dd < catalog::TWEETER_DD { h - inset - r as f64 * GAP } else { inset + r as f64 * GAP };
                    if free(x, y) {
                        return Some((x, y));
                    }
                }
            }
        }
        None
    }

    /// `free_spot`; when the front is full it grows (height, and width for a driver wider than it) until the
    /// driver fits on the centre line. Returns the spot and whether the front changed.
    pub fn make_room(&mut self, dd: f64, outer_d: f64) -> ((f64, f64), bool) {
        if let Some(spot) = self.free_spot(dd, outer_d, false) {
            return (spot, false);
        }
        loop {
            let (w, h) = self.front.size();
            let margin = self.front.edge_radius.max(5.0);
            self.front.corners = Front::rect(w.max(dd.max(outer_d) + 2.0 * margin), h + 10.0, 0.0).corners;
            if let Some(spot) = self.free_spot(dd, outer_d, true) {
                return (spot, true);
            }
        }
    }

    /// Circuits of the stored variants flagged in `mask` (other than the visible one), named "Variant n".
    pub fn variant_projects(&self, mask: &[bool]) -> Vec<(String, Project)> {
        let mut out = vec![];
        for (k, s) in self.variants.slots.iter().enumerate() {
            if let Some(s) = s.as_ref().filter(|s| mask.get(k) == Some(&true) && k != self.variants.active && !s.parts.is_empty()) {
                out.push((format!("Variant {}", k + 1), s.to_project()));
            }
        }
        out
    }

    /// The speaker to evaluate (a file with ways only gets their automatic layout).
    pub fn speaker(&self) -> Speaker {
        let crossover = if self.schematic.parts.is_empty() { schematic::Schematic::from_ways(&self.ways) } else { self.schematic.clone() }.to_project();
        let mut s = Speaker::new(self.front.clone(), crossover);
        s.units = self.units.clone();
        s.axis = self.axis;
        s.distance = self.distance;
        s
    }

    pub fn save(&self, path: &std::path::Path) -> Result<(), String> {
        let json = serde_json::to_string(&FileOut { format: FORMAT, version: FORMAT_VERSION, design: self }).map_err(|e| e.to_string())?;
        std::fs::write(path, json).map_err(|e| format!("{}: {e}", path.display()))
    }

    pub fn load(path: &std::path::Path) -> Result<Design, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        Design::from_json(&text).map_err(|e| format!("{}: {e}", path.display()))
    }

    /// Reads a project file's text; files written before the format header (no `format`) are accepted.
    pub fn from_json(text: &str) -> Result<Design, String> {
        let h: Header = serde_json::from_str(text).map_err(|e| e.to_string())?;
        if h.format.as_deref().is_some_and(|x| x != FORMAT) {
            return Err("not a SoundCAD project".into());
        }
        if h.version > FORMAT_VERSION {
            return Err(format!("made with a newer SoundCAD (file version {}, this program reads up to {FORMAT_VERSION})", h.version));
        }
        serde_json::from_str(text).map_err(|e| e.to_string())
    }
}
