//! SoundCAD: one window for the whole speaker. Top: project file. Left: front, drivers, box and
//! crossover panels; front layout (drag drivers, port and listening axis). Right: live responses.
//! Every edit re-runs `Speaker::evaluate` on a worker thread; the latest result is drawn.
#![cfg_attr(windows, windows_subsystem = "windows")] // no console window beside the app

mod catalog_ui;
mod optimizer_ui;
mod schematic_ui;
mod trace_ui;

use eframe::egui::{self, Align2, Color32, FontId, Pos2, Sense, Stroke, Vec2};
use egui_plot::{Line, Plot, PlotPoints};
use std::path::{Path, PathBuf};
use std::sync::mpsc::{channel, Receiver, Sender};
use soundcad_core::angle::AngleParsing;
use soundcad_core::io::{read_response, MagUnit};
use soundcad_core::Response;
use soundcad_speaker::catalog::{Entry, Kind, DEFAULT_BOX, TWEETER_DD};
use soundcad_speaker::optimize;
use soundcad_speaker::network::{self, Align, Element, Shape, Stage, Way};
use soundcad_speaker::schematic::Schematic;
use soundcad_speaker::{cabinet_analysis, Design, Front, Source, Speaker, Unit, VARIANT_COUNT};
use soundcad_tools::cabinet::{Analysis, Cabinet, MODES_MAX_HZ};
use soundcad_tools::enclosure::{area_diameter, Driver, Enclosure, PassiveRadiator, Radiator};

/// One chart: curves on the result's frequency grid, y range and unit.
struct Chart {
    /// first palette colour (drivers keep the same colour on every chart: Total is colour 0)
    color0: usize,
    title: &'static str,
    unit: &'static str,
    y: (f64, f64),
    curves: Vec<(String, Vec<f64>)>,
}

/// Horizontal directivity map: rows = angles, values dB relative to the on-axis response.
struct Polar {
    angles: Vec<f64>,
    rows: Vec<Vec<f64>>,
}

/// Charts a panel can show; the last one is the directivity map (drawn from `Polar`).
const CHARTS: [&str; 9] = ["SPL", "Phase", "Group delay", "CTA-2034", "Directivity index", "Impedance", "Impedance phase", "Crossover filters", "Directivity map"];
const MAP: usize = 8;
/// Panel grid: 1 or 2 stacked, 4 = 2×2, 6 = 2×3 ; default charts per panel.
const DEFAULT_PANELS: [usize; 6] = [0, 2, 3, 7, MAP, 5];

struct Results {
    freq: Vec<f64>,
    /// one per `CHARTS` entry except the map
    charts: Vec<Chart>,
    polar: Polar,
    seconds: f64,
}

fn compute(s: &Speaker) -> Result<Results, String> {
    let t = std::time::Instant::now();
    let (sim, project) = s.evaluate_project()?;
    let ported = |name: &str| s.units.iter().any(|u| u.name == name && u.port.is_some());
    let (mut spl, mut phase, mut filter) = (vec![], vec![], vec![]);
    let mut both = |label: String, r: &Response| {
        spl.push((label.clone(), r.db()));
        phase.push((label, r.phase_deg()));
    };
    both("Total".into(), &sim.total);
    for (j, r) in sim.driver_spl.iter().enumerate() {
        let name = &s.crossover.parts[sim.circuit.driver_parts[j]].model;
        both(if ported(name) { format!("{name} (cone)") } else { name.clone() }, r);
        filter.push((name.clone(), sim.filter[j].db()));
    }
    // bass-reflex vents as extra sources, like the Enclosure tab: cone, port, total
    for (j, pv) in sim.coupled_sources() {
        both(format!("{} port", s.crossover.parts[sim.circuit.driver_parts[j]].model), &Response::new("", sim.freq.clone(), pv));
    }
    for (name, r) in s.compare_totals(&sim, &project) {
        both(name, &r);
    }
    let gd = vec![("Total".to_string(), soundcad_xover::group_delay(&sim.total).iter().map(|g| g * 1e3).collect())];
    let c = sim.cta();
    let cta = vec![
        ("On axis".to_string(), sim.total.db()),
        ("Listening window".to_string(), c.listening_window.db()),
        ("Early reflections".to_string(), c.early_reflections.db()),
        ("Predicted in-room".to_string(), c.in_room.db()),
        ("Sound power".to_string(), c.power.db()),
    ];
    let di = vec![("Sound power DI".to_string(), c.di.db()), ("Early reflections DI".to_string(), c.er_di.db())];
    let z = &sim.circuit.generator_z;
    let zmag = vec![("|Z|".to_string(), z.iter().map(|z| z.norm()).collect())];
    let zph = vec![("Phase".to_string(), z.iter().map(|z| z.arg().to_degrees()).collect())];
    let axis: Vec<f64> = sim.total.values.iter().map(|v| v.norm()).collect();
    let angles: Vec<f64> = (-18..=18).map(|i| i as f64 * 5.0).collect();
    let rows = angles.iter().map(|&a| sim.pressure(a, 0.0).iter().zip(&axis).map(|(p, a0)| 20.0 * (p.norm() / a0.max(1e-12)).max(1e-6).log10()).collect()).collect();
    let chart = |title, unit, y, curves| Chart { color0: 0, title, unit, y, curves };
    Ok(Results {
        freq: sim.freq.clone(),
        charts: vec![
            chart("SPL at listening axis, 2.83 V", "dB", (50.0, 95.0), spl),
            chart("Phase", "°", (-180.0, 180.0), phase),
            chart("Group delay", "ms", (-1.0, 5.0), gd),
            chart("CTA-2034", "dB", (50.0, 95.0), cta),
            chart("Directivity index", "dB", (-2.0, 15.0), di),
            chart("Impedance", "Ω", (0.0, 20.0), zmag),
            chart("Impedance phase", "°", (-90.0, 90.0), zph),
            Chart { color0: 1, ..chart("Crossover filters (V driver / V amp)", "dB", (-40.0, 5.0), filter) },
        ],
        polar: Polar { angles, rows },
        seconds: t.elapsed().as_secs_f64(),
    })
}
/// What the front editor is dragging.
#[derive(Clone, Copy, PartialEq)]
enum Grab {
    Unit(usize),
    Port(usize),
    Axis,
}

struct App {
    design: Design,
    path: Option<PathBuf>,
    selected: usize,
    grab: Option<Grab>,
    grab_offset: (f64, f64),
    to_worker: Sender<(u64, Speaker)>,
    from_worker: Receiver<(u64, Result<Results, String>)>,
    sent: u64,
    shown: u64,
    results: Option<Results>,
    /// left tab: 0 drivers (with the box), 1 baffle, 2 crossover
    tab: usize,
    /// number of chart panels (1…6) and the chart in each
    panels: usize,
    panel_chart: [usize; 6],
    /// quick filter form: shape, order, alignment, fc, fc2 (band-pass), nominal load Ω (None = from driver)
    quick: (Shape, u32, Align, f64, f64, Option<f64>),
    /// graphic crossover editor state
    xo: schematic_ui::Editor,
    title: String,
    /// optimizer: running job, undo stack (component, previous value), message, target drawn in the SPL chart
    opt_rx: Option<Receiver<Result<(Vec<usize>, optimize::Outcome), String>>>,
    opt_undo: Vec<Vec<(usize, f64)>>,
    opt_status: String,
    show_target: bool,
    opt_open: bool,
    /// chart smoothing: 0 = none, else 1/N octave (all magnitude charts); cache of the smoothed curves
    smooth: u32,
    smooth_cache: std::collections::HashMap<(u64, u32, usize), Vec<(String, Vec<f64>)>>,
    /// crossover variants: other variants drawn in the SPL chart, pending overwrite / delete confirmation (delete?, slot)
    variant_show: [bool; VARIANT_COUNT],
    variant_confirm: Option<(bool, usize)>,
    /// driver library, shown instead of the project when `show_catalog`; `picking` = add-from-catalog pop-up
    catalog: catalog_ui::CatalogUi,
    show_catalog: bool,
    picking: bool,
    /// Drivers tab: unit waiting for the remove confirmation, unit being replaced from the catalog
    confirm_remove: Option<usize>,
    replacing: Option<usize>,
    /// directivity map image of the shown result
    map_cache: Option<(u64, egui::TextureHandle)>,
    error: Option<String>,
    status: String,
    dirty: bool,
}

/// Placeholder T/S for a new driver (SB Acoustics SB15NBAC30-8).
fn default_ts() -> Driver {
    Driver {
        manufacturer: "SB ACOUSTICS".into(),
        model: "SB15NBAC30-8".into(),
        re: 5.7,
        le: 13.9651933646471,
        leb: 0.0,
        ke: 0.020500583583649,
        rss: 13699.8274864857,
        rms: 0.6,
        mms: 10.0,
        cms: 1.81,
        sd: 82.0,
        bl: 5.7,
    }
}

fn read(path: &Path, unit: MagUnit) -> Result<Response, String> {
    read_response(path, unit, &AngleParsing::default()).map_err(|e| format!("{}: {e}", path.display()))
}

/// Response file chosen by the user (dB: frequency response, linear: impedance); `None` when the dialog is cancelled.
fn pick_response(unit: MagUnit) -> Option<Result<Response, String>> {
    let (what, exts): (&str, &[&str]) = match unit {
        MagUnit::Db => ("Frequency response", &["txt", "frd"]),
        MagUnit::Linear => ("Impedance", &["zma", "txt"]),
    };
    let p = rfd::FileDialog::new().add_filter(what, exts).pick_file()?;
    Some(read(&p, unit))
}

/// Button that loads a response file; a read error goes to `status`.
fn load_button(ui: &mut egui::Ui, label: &str, unit: MagUnit, status: &mut Option<String>) -> Option<Response> {
    match ui.button(label).clicked().then(|| pick_response(unit)).flatten()? {
        Ok(r) => Some(r),
        Err(e) => {
            *status = Some(e);
            None
        }
    }
}

/// Polar files chosen by the user (`read_polars`); `None` when the dialog is cancelled.
fn pick_polars() -> Option<Result<(String, soundcad_xover::DriverResponses), String>> {
    let paths = rfd::FileDialog::new().add_filter("Frequency response", &["txt", "frd"]).pick_files()?;
    Some(read_polars(&paths))
}

/// Demo curves built into the program (SB Acoustics datasheet traces).
fn demo_curve(file: &str, unit: MagUnit) -> Result<Response, String> {
    let text = match file {
        "SB15NBAC30-8_hor_deg+0.txt" => include_str!("../demo/SB15NBAC30-8_hor_deg+0.txt"),
        "SB15NBAC30-8_ZR.txt" => include_str!("../demo/SB15NBAC30-8_ZR.txt"),
        "SB26STAC-C000-4_hor_deg+0.txt" => include_str!("../demo/SB26STAC-C000-4_hor_deg+0.txt"),
        "SB26STAC-C000-4_ZR.txt" => include_str!("../demo/SB26STAC-C000-4_ZR.txt"),
        _ => return Err(format!("no demo curve {file}")),
    };
    soundcad_core::io::parse_response(file.trim_end_matches(".txt"), text, unit).map_err(|e| format!("{file}: {e}"))
}

/// Polar files of one driver measured in its cabinet: the unit name (common start of the file names) and the sets.
fn read_polars(paths: &[PathBuf]) -> Result<(String, soundcad_xover::DriverResponses), String> {
    let files = paths.iter().map(|p| read(p, MagUnit::Db)).collect::<Result<Vec<_>, _>>()?;
    let name = soundcad_speaker::polar::common_name(&files.iter().map(|r| r.name.clone()).collect::<Vec<_>>()).unwrap_or_else(|| "Driver".into());
    Ok((name, soundcad_speaker::polar::from_measurements(files, None)?))
}

/// Built-in datasheet curves of driver `model` (`<model>_hor_deg+0.txt`, `<model>_ZR.txt`) as a half-space source.
fn demo_source(model: &str, box_correction: Option<(Driver, Radiator)>) -> Result<Source, String> {
    let (spl, z) = (demo_curve(&format!("{model}_hor_deg+0.txt"), MagUnit::Db)?, demo_curve(&format!("{model}_ZR.txt"), MagUnit::Linear)?);
    Ok(Source::HalfSpace { response: spl, impedance: Some(z), box_correction })
}

/// Studio monitor from the user's first crossover round: 180×330 mm closed box, SB15 + SB26 datasheet curves.
fn demo() -> Result<Design, String> {
    let woofer = Unit { z: 15.0, ..Unit::new("SB15NBAC30-8", (90.0, 120.0), 102.0, demo_source("SB15NBAC30-8", Some((default_ts(), Radiator::Closed { vb: 8.0, qa: 5.0, ql: 10.0 })))?) };
    let tweeter = Unit::new("SB26STAC-C000-4", (90.0, 250.0), 28.0, demo_source("SB26STAC-C000-4", None)?);
    let mut tw = Way::high_pass_2("SB26STAC-C000-4", 8.0, 0.55);
    tw.stages.insert(1, Stage::Series(Element::R(3.3)));
    Ok(Design { units: vec![woofer, tweeter], ways: vec![Way::low_pass_2("SB15NBAC30-8", 0.47, 8.2), tw], ..Design::new(Front::rect(180.0, 330.0, 10.0), (90.0, 250.0)) })
}

/// Number of vents of a bass-reflex unit (1 otherwise: n passive radiators are drawn as one of their total area).
fn vent_count(u: &Unit) -> u32 {
    match &u.source {
        Source::Simulated { radiator: Radiator::BassReflex { n, .. }, .. } | Source::HalfSpace { box_correction: Some((_, Radiator::BassReflex { n, .. })), .. } => (*n).max(1),
        _ => 1,
    }
}

/// Load for the quick filter: minimum |Z| above 100 Hz of the impedance data, or Re × 1.15 from T/S, or 8 Ω.
fn nominal_impedance(u: &Unit) -> f64 {
    let min_above = |z: &Response| z.freq.iter().zip(&z.values).filter(|(f, _)| **f > 100.0).map(|(_, v)| v.norm()).fold(f64::INFINITY, f64::min);
    match &u.source {
        Source::HalfSpace { impedance: Some(z), .. } if !z.freq.is_empty() => min_above(z),
        Source::HalfSpace { box_correction: Some((d, _)), .. } | Source::Simulated { driver: d, .. } => d.re * 1.15,
        Source::InCabinet { responses } => responses.impedance.as_ref().map_or(8.0, min_above),
        _ => 8.0,
    }
}

/// Three-way starting point: woofer in bass reflex, mid in its own closed chamber, SB26 tweeter
/// (woofer and mid are the SB15 T/S model: replace them with your drivers).
fn demo_3way() -> Result<Design, String> {
    let ts = |name: &str, y, radiator| Unit::new(name, (110.0, y), 102.0, Source::Simulated { driver: default_ts(), radiator });
    let woofer = Unit { port: Some((110.0, 45.0, 50.0)), ..ts("Woofer", 130.0, Radiator::BassReflex { vb: 18.0, qa: 10.0, ql: 15.0, diam: 5.0, len: 12.0, qp: 100.0, n: 1, end_corr: 0.85 }) };
    let mid = ts("Mid", 290.0, Radiator::Closed { vb: 3.0, qa: 5.0, ql: 10.0 });
    let tweeter = Unit::new("Tweeter", (110.0, 400.0), 28.0, demo_source("SB26STAC-C000-4", None)?);
    let way = |driver: &str, shape, fc, fc2, r, inverted| Way { driver: driver.into(), stages: network::filter_stages(shape, 2, Align::Butterworth, fc, fc2, r), inverted };
    let mut tw = way("Tweeter", Shape::HighPass, 3000.0, 0.0, 4.0, false);
    tw.stages.insert(0, Stage::Series(Element::R(2.2)));
    let ways = vec![way("Woofer", Shape::LowPass, 400.0, 0.0, 6.5, false), way("Mid", Shape::BandPass, 400.0, 3000.0, 6.5, true), tw];
    Ok(Design { units: vec![woofer, mid, tweeter], ways, ..Design::new(Front::rect(220.0, 460.0, 15.0), (110.0, 400.0)) })
}

/// Older projects and the demos only have ladder ways: draw them as a schematic.
fn with_schematic(mut d: Design) -> Design {
    if d.schematic.parts.is_empty() && !d.ways.is_empty() {
        d.schematic = Schematic::from_ways(&d.ways);
    }
    d
}

fn empty_design() -> Design {
    Design::new(Front::rect(200.0, 300.0, 10.0), (100.0, 200.0))
}

/// Heading of a group of controls in the left panel.
fn section(ui: &mut egui::Ui, title: &str) {
    ui.add_space(6.0);
    ui.label(egui::RichText::new(title).strong());
    ui.separator();
}

/// Two arrows chasing each other in a circle, drawn by hand (the font has no such glyph).
fn replace_button(ui: &mut egui::Ui) -> egui::Response {
    let (rect, resp) = ui.allocate_exact_size(Vec2::splat(18.0), Sense::click());
    let v = ui.style().interact(&resp);
    let p = ui.painter();
    p.rect_filled(rect, 3.0, v.weak_bg_fill);
    let (c, r, stroke) = (rect.center(), 5.0, egui::Stroke::new(1.3_f32, v.fg_stroke.color));
    for start in [0.3, 0.3 + std::f32::consts::PI] {
        let arc: Vec<egui::Pos2> = (0..=8).map(|k| c + Vec2::angled(start + k as f32 / 8.0 * 2.2) * r).collect();
        let (tip, along) = (arc[8], Vec2::angled(start + 2.2 + std::f32::consts::FRAC_PI_2));
        let side = along.rot90();
        p.add(egui::Shape::line(arc, stroke));
        p.add(egui::Shape::line(vec![tip - along * 2.5 + side * 2.5, tip, tip - along * 2.5 - side * 2.5], stroke));
    }
    resp
}

fn project_dialog() -> rfd::FileDialog {
    rfd::FileDialog::new().add_filter("SoundCAD project", &["soundcad"])
}

impl App {
    fn new(cc: &eframe::CreationContext) -> Self {
        let (to_worker, jobs) = channel::<(u64, Speaker)>();
        let (done, from_worker) = channel();
        let ctx = cc.egui_ctx.clone();
        std::thread::spawn(move || {
            while let Ok(mut job) = jobs.recv() {
                // only the newest request matters
                while let Ok(j) = jobs.try_recv() {
                    job = j;
                }
                // a bug in the chain must not kill the worker (the window would stop updating)
                let r = std::panic::catch_unwind(|| compute(&job.1)).unwrap_or_else(|_| Err("internal error in the calculation (see console)".into()));
                if done.send((job.0, r)).is_err() {
                    break;
                }
                ctx.request_repaint();
            }
        });
        let (design, status) = match demo() {
            Ok(d) => (with_schematic(d), "Demo project: studio monitor".to_string()),
            Err(e) => (empty_design(), format!("demo project: {e}")),
        };
        App { design, path: None, selected: 0, grab: None, grab_offset: (0.0, 0.0), to_worker, from_worker, sent: 0, shown: 0, results: None, tab: 0, panels: 6, panel_chart: DEFAULT_PANELS, quick: (Shape::LowPass, 2, Align::Butterworth, 2500.0, 2500.0, None), xo: Default::default(), title: String::new(), opt_rx: None, opt_undo: vec![], opt_status: String::new(), show_target: true, opt_open: false, smooth: 0, smooth_cache: Default::default(), variant_show: [false; VARIANT_COUNT], variant_confirm: None, catalog: catalog_ui::CatalogUi::load(), show_catalog: false, picking: false, confirm_remove: None, replacing: None, map_cache: None, error: None, status, dirty: true }
    }

    fn submit(&mut self) {
        self.dirty = false;
        if self.design.units.is_empty() {
            (self.results, self.error) = (None, None);
            return;
        }
        self.sent += 1;
        let mut speaker = self.design.speaker();
        speaker.compare = self.design.variant_projects(&self.variant_show);
        let _ = self.to_worker.send((self.sent, speaker));
    }

    fn set_design(&mut self, d: Design, path: Option<PathBuf>) {
        // chart layout saved with the project (checked: a hand-edited file must not break the window)
        if let Some(v) = &d.view {
            if [1, 2, 4, 6].contains(&v.panels) && v.charts.iter().all(|&c| c < CHARTS.len()) {
                (self.panels, self.panel_chart) = (v.panels, v.charts);
            }
            if [0, 3, 6, 12, 24].contains(&v.smoothing) {
                self.smooth = v.smoothing;
            }
        }
        self.design = with_schematic(d);
        self.xo = Default::default();
        self.path = path;
        self.selected = 0;
        (self.confirm_remove, self.replacing) = (None, None);
        self.results = None;
        self.dirty = true;
    }

    fn save_to(&mut self, path: PathBuf) {
        self.design.sync_variants();
        self.design.view = Some(soundcad_speaker::View { panels: self.panels, charts: self.panel_chart, smoothing: self.smooth });
        let path = if path.extension().is_none() { path.with_extension("soundcad") } else { path };
        match self.design.save(&path) {
            Ok(()) => {
                self.status = format!("Saved {}", path.display());
                self.path = Some(path);
            }
            Err(e) => self.status = format!("Save failed: {e}"),
        }
    }

    fn top_bar(&mut self, ui: &mut egui::Ui) {
        egui::menu::bar(ui, |ui| {
            ui.menu_button("File", |ui| {
                // every file action goes back to the project view
                let mut file_action = false;
                if ui.button("New").clicked() {
                    self.set_design(empty_design(), None);
                    self.status = "New project".into();
                    file_action = true;
                }
                if ui.button("Open…").clicked() {
                    file_action = true;
                    if let Some(p) = project_dialog().pick_file() {
                        match Design::load(&p) {
                            Ok(d) => {
                                self.status = format!("Opened {}", p.display());
                                self.set_design(d, Some(p));
                            }
                            Err(e) => self.status = format!("Open failed: {e}"),
                        }
                    }
                }
                if ui.button("Save").clicked() {
                    file_action = true;
                    match self.path.clone() {
                        Some(p) => self.save_to(p),
                        None => {
                            if let Some(p) = project_dialog().set_file_name("speaker.soundcad").save_file() {
                                self.save_to(p);
                            }
                        }
                    }
                }
                if ui.button("Save as…").clicked() {
                    file_action = true;
                    if let Some(p) = project_dialog().set_file_name("speaker.soundcad").save_file() {
                        self.save_to(p);
                    }
                }
                ui.separator();
                ui.menu_button("Examples", |ui| {
                    for (label, make) in [("Demo 2-way", demo as fn() -> Result<Design, String>), ("Demo 3-way", demo_3way)] {
                        if ui.button(label).clicked() {
                            file_action = true;
                            match make() {
                                Ok(d) => {
                                    self.status = format!("{label} loaded");
                                    self.set_design(d, None);
                                }
                                Err(e) => self.status = e,
                            }
                        }
                    }
                });
                if file_action {
                    self.show_catalog = false;
                    ui.close_menu();
                }
            });
            ui.menu_button("Database", |ui| {
                if ui.selectable_label(self.show_catalog, "Driver catalog").clicked() {
                    self.show_catalog = !self.show_catalog;
                    ui.close_menu();
                }
                ui.separator();
                let mut used = self.catalog.menu_records(ui);
                if ui.add_enabled(!self.design.units.is_empty(), egui::Button::new("Save selected project driver to catalog")).clicked() {
                    self.save_unit_to_catalog();
                    ui.close_menu();
                }
                ui.separator();
                used |= self.catalog.menu_files(ui);
                if used {
                    self.show_catalog = true;
                    ui.close_menu();
                }
            });
            ui.menu_button("Options", |ui| {
                ui.label("Number of charts");
                for n in [1, 2, 4, 6] {
                    if ui.radio_value(&mut self.panels, n, n.to_string()).clicked() {
                        ui.close_menu();
                    }
                }
            });
            ui.separator();
            ui.label(if self.show_catalog { &self.catalog.status } else { &self.status });
        });
    }

    /// Front layout: baffle, drivers, ports and listening axis, drag to move (mm, origin bottom-left).
    fn front_editor(&mut self, ui: &mut egui::Ui) {
        let size = Vec2::new(ui.available_width(), ui.available_height().min(ui.available_width() * 1.4));
        let (rect, resp) = ui.allocate_exact_size(size, Sense::click_and_drag());
        let p = ui.painter_at(rect);
        let (w, h) = self.design.front.size();
        let scale = ((rect.width() - 40.0) as f64 / w).min((rect.height() - 40.0) as f64 / h);
        let o = Pos2::new(rect.center().x - (w * scale / 2.0) as f32, rect.center().y + (h * scale / 2.0) as f32);
        let to_screen = |x: f64, y: f64| Pos2::new(o.x + (x * scale) as f32, o.y - (y * scale) as f32);
        let to_mm = |q: Pos2| (((q.x - o.x) as f64 / scale).round(), ((o.y - q.y) as f64 / scale).round());

        let baffle = egui::Rect::from_two_pos(to_screen(0.0, 0.0), to_screen(w, h));
        p.rect_filled(baffle, (self.design.front.edge_radius * scale) as f32, Color32::from_gray(60));
        p.text(baffle.center_bottom() + Vec2::new(0.0, 14.0), Align2::CENTER_CENTER, format!("{w:.0} × {h:.0} mm"), FontId::proportional(12.0), Color32::GRAY);
        for (i, u) in self.design.units.iter().enumerate() {
            let c = to_screen(u.x, u.y);
            let col = if i == self.selected { Color32::from_rgb(90, 160, 240) } else { Color32::from_gray(150) };
            if u.outer_d > u.dd {
                p.circle_filled(c, (u.outer_d / 2.0 * scale) as f32, Color32::from_gray(42));
                p.circle_stroke(c, (u.outer_d / 2.0 * scale) as f32, Stroke::new(1.0_f32, col));
            }
            p.circle_filled(c, (u.dd / 2.0 * scale) as f32, Color32::from_gray(25));
            p.circle_stroke(c, (u.dd / 2.0 * scale) as f32, Stroke::new(2.0_f32, col));
            p.text(c, Align2::CENTER_CENTER, &u.name, FontId::proportional(11.0), col);
            if let Some((px, py, pd)) = u.port {
                // n vents side by side, round or rectangular, true size
                let (vw, vh) = u.port_rect.unwrap_or((pd, pd));
                let n = vent_count(u);
                let orange = Stroke::new(1.5_f32, Color32::from_rgb(200, 150, 60));
                for k in 0..n {
                    let cx = px + (k as f64 - (n as f64 - 1.0) / 2.0) * (vw + 10.0);
                    let pc = to_screen(cx, py);
                    if u.port_rect.is_some() {
                        let r = egui::Rect::from_center_size(pc, Vec2::new((vw * scale) as f32, (vh * scale) as f32));
                        p.rect_filled(r, 2.0, Color32::BLACK);
                        p.rect_stroke(r, 2.0, orange, egui::StrokeKind::Middle);
                    } else {
                        p.circle_filled(pc, (pd / 2.0 * scale) as f32, Color32::BLACK);
                        p.circle_stroke(pc, (pd / 2.0 * scale) as f32, orange);
                    }
                }
            }
        }
        let a = to_screen(self.design.axis.0, self.design.axis.1);
        let red = Color32::from_rgb(230, 70, 70);
        p.line_segment([a - Vec2::new(10.0, 0.0), a + Vec2::new(10.0, 0.0)], Stroke::new(1.5_f32, red));
        p.line_segment([a - Vec2::new(0.0, 10.0), a + Vec2::new(0.0, 10.0)], Stroke::new(1.5_f32, red));

        let design = &self.design;
        let under = |q: Pos2| {
            let near = |x: f64, y: f64, r: f64| to_screen(x, y).distance(q) <= (r * scale) as f32 + 4.0;
            // drivers and ports first: the axis often sits on the tweeter centre (it also has fields)
            if let Some(i) = design.units.iter().position(|u| u.port.is_some_and(|(x, y, d)| near(x, y, d / 2.0))) {
                Some(Grab::Port(i))
            } else if let Some((i, _)) = design.units.iter().enumerate().filter(|(_, u)| near(u.x, u.y, u.footprint() / 2.0)).min_by(|a, b| to_screen(a.1.x, a.1.y).distance(q).total_cmp(&to_screen(b.1.x, b.1.y).distance(q))) {
                // the closest centre: frames may overlap while the user moves things around
                Some(Grab::Unit(i))
            } else {
                (to_screen(design.axis.0, design.axis.1).distance(q) < 12.0).then_some(Grab::Axis)
            }
        };
        let clicked = resp.clicked().then(|| resp.interact_pointer_pos().and_then(&under)).flatten();
        // egui reports the drag after a few pixels of movement: hit-test where the button went down
        let pressed = resp.drag_started().then(|| ui.input(|i| i.pointer.press_origin()).or(resp.interact_pointer_pos())).flatten();
        let grabbed = pressed.and_then(&under);
        if let Some(Grab::Unit(i) | Grab::Port(i)) = clicked {
            self.selected = i;
        }
        if resp.drag_started() {
            if let Some(q) = pressed {
                self.grab = grabbed;
                if let Some(Grab::Unit(i) | Grab::Port(i)) = self.grab {
                    self.selected = i;
                }
                // what is grabbed keeps its distance from the pointer: a frame is grabbed far from its centre
                let at = match self.grab {
                    Some(Grab::Unit(i)) => (self.design.units[i].x, self.design.units[i].y),
                    Some(Grab::Port(i)) => self.design.units[i].port.map_or((0.0, 0.0), |p| (p.0, p.1)),
                    _ => self.design.axis,
                };
                let m = to_mm(q);
                self.grab_offset = (at.0 - m.0, at.1 - m.1);
            }
        }
        if resp.dragged() {
            if let (Some(g), Some(q)) = (self.grab, resp.interact_pointer_pos()) {
                let (x, y) = to_mm(q);
                let (x, y) = ((x + self.grab_offset.0).round().clamp(0.0, w), (y + self.grab_offset.1).round().clamp(0.0, h));
                match g {
                    Grab::Unit(i) => (self.design.units[i].x, self.design.units[i].y) = (x, y),
                    Grab::Port(i) => {
                        if let Some(pt) = &mut self.design.units[i].port {
                            (pt.0, pt.1) = (x, y);
                        }
                    }
                    Grab::Axis => self.design.axis = (x, y),
                }
                self.dirty = true;
            }
        }
        if resp.drag_stopped() {
            self.grab = None;
        }
    }

    /// Left third: one tab at a time (drivers and box, baffle, crossover).
    fn left_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            for (i, t) in ["Drivers", "Baffle", "Crossover"].iter().enumerate() {
                ui.selectable_value(&mut self.tab, i, egui::RichText::new(*t).heading());
            }
        });
        ui.separator();
        let changed = match self.tab {
            0 => egui::ScrollArea::vertical().show(ui, |ui| self.drivers_tab(ui)).inner,
            1 => self.baffle_tab(ui),
            // the drawing needs the whole height: no scroll area
            _ => self.crossover_tab(ui),
        };
        self.sync_ports();
        if changed {
            self.dirty = true;
        }
    }

    /// Drivers of the speaker (T/S and responses come from the catalog) and, below, the box of the selected one.
    fn drivers_tab(&mut self, ui: &mut egui::Ui) -> bool {
        let mut changed = false;
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new("Drivers").strong());
            ui.menu_button("+", |ui| {
                if ui.button("From catalog…").clicked() {
                    self.picking = true;
                    ui.close_menu();
                }
                if ui.button("From response file…").clicked() {
                    ui.close_menu();
                    match pick_response(MagUnit::Db) {
                        Some(Ok(r)) => {
                            let name = if r.name.is_empty() { "Driver".to_string() } else { r.name.clone() };
                            changed |= self.add_unit(name, Source::HalfSpace { response: r, impedance: None, box_correction: None }, 100.0, 0.0);
                        }
                        Some(Err(e)) => self.status = e,
                        None => {}
                    }
                }
                if ui.button("From polar files…").on_hover_text("One file per angle, measured in the finished cabinet: name hor -30.txt, name ver 20.txt …").clicked() {
                    ui.close_menu();
                    match pick_polars() {
                        Some(Ok((name, responses))) => {
                            self.status = format!("{name}: {}", soundcad_speaker::polar::summary(&responses));
                            changed |= self.add_unit(name, Source::InCabinet { responses }, 100.0, 0.0);
                        }
                        Some(Err(e)) => self.status = e,
                        None => {}
                    }
                }
            }).response.on_hover_text("Add a driver");
        });
        ui.separator();
        if self.design.units.is_empty() {
            ui.label("No drivers yet: press + to take one from the catalog (Database > Add driver creates new ones).");
            return changed;
        }
        self.selected = self.selected.min(self.design.units.len() - 1);
        let mut remove = None;
        egui::ScrollArea::vertical().id_salt("unitlist").max_height(190.0).auto_shrink([false, true]).show(ui, |ui| {
            for (i, u) in self.design.units.iter().enumerate() {
                let box_name = match &u.source {
                    Source::Simulated { radiator, .. } | Source::HalfSpace { box_correction: Some((_, radiator)), .. } => BOX_TYPES[box_kind(radiator)],
                    Source::HalfSpace { .. } => "response as measured",
                    Source::InCabinet { .. } => "measured in cabinet",
                };
                ui.horizontal(|ui| {
                    ui.label((i + 1).to_string());
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.small_button("x").on_hover_text("Remove this driver").clicked() {
                            self.confirm_remove = Some(i);
                        }
                        if replace_button(ui).on_hover_text("Replace with another driver from the catalog: position, depth, box and crossover stay").clicked() {
                            self.replacing = Some(i);
                            self.picking = false;
                        }
                        ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                            if ui.selectable_label(i == self.selected, format!("{}  ·  {box_name}", u.name)).clicked() {
                                self.selected = i;
                            }
                        });
                    });
                });
                if self.confirm_remove == Some(i) {
                    ui.horizontal(|ui| {
                        ui.label(format!("Remove {} and its crossover branch?", u.name));
                        if ui.button("Keep").clicked() {
                            self.confirm_remove = None;
                        }
                        if ui.button("Remove").clicked() {
                            remove = Some(i);
                        }
                    });
                }
            }
        });
        if let Some(i) = remove {
            self.design.remove_unit(i);
            // indices shift: nothing may keep pointing into the old lists
            (self.confirm_remove, self.replacing) = (None, None);
            self.xo = Default::default();
            self.opt_undo.clear();
            if i < self.selected {
                self.selected -= 1;
            }
            changed = true;
            if self.design.units.is_empty() {
                return changed;
            }
            self.selected = self.selected.min(self.design.units.len() - 1);
        }
        egui::CollapsingHeader::new("Name, placement and response source").id_salt("unitdetails").default_open(false).show(ui, |ui| {
            changed |= self.unit_panel(ui);
        });
        section(ui, &format!("Box of {}", self.design.units[self.selected].name));
        changed |= self.box_tab(ui);
        changed
    }
    /// Box of the selected driver (T/S model, or box correction of a measured response) and its port.
    fn box_tab(&mut self, ui: &mut egui::Ui) -> bool {
        let mut changed = false;
        let analysis = cabinet_analysis(&self.design.front, &self.design.units, self.selected);
        let Some(u) = self.design.units.get_mut(self.selected) else {
            ui.label("No driver yet.");
            return false;
        };
        let Unit { source, port, port_rect, cabinet, .. } = u;
        let (driver, radiator) = match source {
            Source::Simulated { driver, radiator } => (Some(&*driver), Some(radiator)),
            Source::HalfSpace { box_correction: Some((driver, radiator)), .. } => (Some(&*driver), Some(radiator)),
            _ => (None, None),
        };
        let Some(r) = radiator else {
            if matches!(self.design.units[self.selected].source, Source::InCabinet { .. }) {
                ui.label("Measured in the finished cabinet: box and baffle are part of the measurement.");
            } else {
                ui.label("This driver has no box model (e.g. a tweeter with its own closed back).");
                ui.label("For a measured response, enable “Box correction from T/S” under “Name, placement and response source”.");
            }
            return false;
        };
        // with a cabinet set, volume and Qa are results: shown here, and the simulation takes them from there
        if let Some(a) = &analysis {
            *r = a.fit(*r);
        }
        changed |= box_editor(ui, r, analysis.is_some());
        if let (Some(d), false) = (driver, matches!(r, Radiator::InfiniteBaffle)) {
            if let Some(fb) = (Enclosure { driver: d.clone(), radiator: *r, eg: 2.83, modes: vec![] }).simulate(&[100.0]).fb {
                ui.label(format!("Fb {fb:.1} Hz"));
            }
        }
        if !matches!(r, Radiator::InfiniteBaffle) {
            changed |= cabinet_editor(ui, cabinet, analysis.as_ref());
        }
        if let (Radiator::Passive { pr, .. }, Some((px, py, _))) = (&mut *r, &mut *port) {
            let passives: Vec<_> = self.catalog.cat.entries.iter().filter(|e| e.kind == Kind::Passive).collect();
            egui::ComboBox::from_id_salt("prcat").selected_text("Get parameters from catalog…").show_ui(ui, |ui| {
                if passives.is_empty() {
                    ui.label("No passive radiator in the catalog (Database menu).");
                }
                for e in passives {
                    if ui.selectable_label(false, e.title()).clicked() {
                        if let Some(p) = e.passive(pr.n, pr.extra) {
                            *pr = p;
                            changed = true;
                        }
                    }
                }
            });
            egui::Grid::new("prpos").num_columns(2).show(ui, |ui| {
                ui.label("Position X");
                changed |= ui.add(egui::DragValue::new(px).suffix(" mm")).changed();
                ui.end_row();
                ui.label("Position Y");
                changed |= ui.add(egui::DragValue::new(py).suffix(" mm")).changed();
                ui.end_row();
            });
            ui.label(egui::RichText::new("Several radiators are modelled as one source of the same total area.").weak());
        }
        if let (Radiator::BassReflex { diam, n, len, .. }, Some((px, py, _))) = (r, port) {
            section(ui, "Vent");
            ui.horizontal(|ui| {
                let mut rect = port_rect.is_some();
                changed |= ui.selectable_value(&mut rect, false, "Round").changed();
                changed |= ui.selectable_value(&mut rect, true, "Rectangular").changed();
                match (rect, port_rect.is_some()) {
                    (true, false) => *port_rect = Some((*diam * 10.0 * 1.5, *diam * 10.0 * 0.6)),
                    (false, true) => *port_rect = None,
                    _ => {}
                }
            });
            egui::Grid::new("vent").num_columns(2).show(ui, |ui| {
                match port_rect {
                    Some((w, h)) => {
                        ui.label("Width");
                        changed |= ui.add(egui::DragValue::new(w).range(5.0..=500.0).suffix(" mm")).changed();
                        ui.end_row();
                        ui.label("Height");
                        changed |= ui.add(egui::DragValue::new(h).range(5.0..=500.0).suffix(" mm")).changed();
                        ui.end_row();
                    }
                    None => {
                        let mut d = *diam * 10.0;
                        ui.label("Diameter");
                        if ui.add(egui::DragValue::new(&mut d).range(5.0..=500.0).suffix(" mm")).changed() {
                            *diam = d / 10.0;
                            changed = true;
                        }
                        ui.end_row();
                    }
                }
                ui.label("Length");
                changed |= ui.add(egui::DragValue::new(len).speed(0.1).range(0.1..=1000.0).suffix(" cm")).changed();
                ui.end_row();
                ui.label("Number of vents");
                changed |= ui.add(egui::DragValue::new(n).range(1..=4)).changed();
                ui.end_row();
                ui.label("Position X");
                changed |= ui.add(egui::DragValue::new(px).suffix(" mm")).changed();
                ui.end_row();
                ui.label("Position Y");
                changed |= ui.add(egui::DragValue::new(py).suffix(" mm")).changed();
                ui.end_row();
            });
            if port_rect.is_some() {
                ui.label(egui::RichText::new("Tuning uses the round vent of equal area (approximation).").weak());
            }
        }
        changed
    }
    /// Front size, listening axis and the drag-and-drop layout.
    fn baffle_tab(&mut self, ui: &mut egui::Ui) -> bool {
        let mut changed = false;
        let (mut w, mut h) = self.design.front.size();
        section(ui, "Cabinet front");
        let mut size = false;
        ui.horizontal(|ui| {
            ui.label("Width");
            size |= ui.add(egui::DragValue::new(&mut w).range(50.0..=2000.0).suffix(" mm")).changed();
            ui.label("Height");
            size |= ui.add(egui::DragValue::new(&mut h).range(50.0..=2000.0).suffix(" mm")).changed();
            ui.label("Edge radius");
            changed |= ui.add(egui::DragValue::new(&mut self.design.front.edge_radius).range(0.0..=100.0).suffix(" mm")).changed();
        });
        if size {
            // the outline only: edge radius and edge type stay
            self.design.front.corners = Front::rect(w, h, 0.0).corners;
            changed = true;
        }
        section(ui, "Listening position");
        ui.horizontal(|ui| {
            ui.label("Axis X");
            changed |= ui.add(egui::DragValue::new(&mut self.design.axis.0).suffix(" mm")).changed();
            ui.label("Axis Y");
            changed |= ui.add(egui::DragValue::new(&mut self.design.axis.1).suffix(" mm")).changed();
            ui.label("Distance");
            changed |= ui.add(egui::DragValue::new(&mut self.design.distance).range(300.0..=10000.0).speed(10.0).suffix(" mm")).changed();
        });
        section(ui, "Layout");
        if !self.design.units.is_empty() {
            self.selected = self.selected.min(self.design.units.len() - 1);
            let u = &mut self.design.units[self.selected];
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(u.name.clone()).strong());
                ui.label("X");
                changed |= ui.add(egui::DragValue::new(&mut u.x).suffix(" mm")).changed();
                ui.label("Y");
                changed |= ui.add(egui::DragValue::new(&mut u.y).suffix(" mm")).changed();
            });
        }
        ui.label(egui::RichText::new("Click to select, drag to move · inner circle = radiating surface, outer = frame · ✚ = listening axis").weak());
        self.front_editor(ui);
        changed
    }

    fn crossover_tab(&mut self, ui: &mut egui::Ui) -> bool {
        if self.design.units.is_empty() {
            ui.label("No driver yet.");
            return false;
        }
        self.selected = self.selected.min(self.design.units.len() - 1);
        let u = &self.design.units[self.selected];
        let name = u.name.clone();
        let nominal = nominal_impedance(u);
        let mut changed = false;
        section(ui, "Tools");
        egui::CollapsingHeader::new("Quick filter (draws a driver's branch with computed values)").id_salt("quickfilter").default_open(false).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Driver");
                egui::ComboBox::from_id_salt("selected").selected_text(name.clone()).show_ui(ui, |ui| {
                    for (i, u) in self.design.units.iter().enumerate() {
                        ui.selectable_value(&mut self.selected, i, &u.name);
                    }
                });
            });
            let (shape, order, align, fc, fc2, r) = &mut self.quick;
            ui.horizontal(|ui| {
                for (s, label) in [(Shape::LowPass, "Low-pass (woofer)"), (Shape::BandPass, "Band-pass (mid)"), (Shape::HighPass, "High-pass (tweeter)")] {
                    ui.selectable_value(shape, s, label);
                }
            });
            ui.horizontal(|ui| {
                ui.label("Order");
                for n in 1..=3 {
                    ui.selectable_value(order, n, n.to_string());
                }
                if *order == 2 {
                    ui.separator();
                    ui.selectable_value(align, Align::Butterworth, "Butterworth");
                    ui.selectable_value(align, Align::LinkwitzRiley, "Linkwitz-Riley");
                }
            });
            egui::Grid::new("quick").num_columns(2).show(ui, |ui| {
                ui.label(if *shape == Shape::BandPass { "Lower cut-off" } else { "Cut-off" });
                ui.add(egui::DragValue::new(fc).speed(10.0).range(20.0..=20000.0).suffix(" Hz"));
                ui.end_row();
                if *shape == Shape::BandPass {
                    ui.label("Upper cut-off");
                    ui.add(egui::DragValue::new(fc2).speed(10.0).range(20.0..=20000.0).suffix(" Hz"));
                    ui.end_row();
                }
                ui.label("Load (nominal Z)");
                let mut z = r.unwrap_or(nominal);
                if ui.add(egui::DragValue::new(&mut z).speed(0.1).range(1.0..=64.0).suffix(" Ω")).changed() {
                    *r = Some(z);
                }
                ui.end_row();
            });
            if ui.button(format!("Apply to {name} (redraws its branch)")).clicked() {
                let stages = network::filter_stages(*shape, *order, *align, *fc, *fc2, r.unwrap_or(nominal));
                if let Some(w) = self.design.ways.iter_mut().find(|w| w.driver == name) {
                    w.stages = stages;
                    self.design.schematic.replace_branch(w);
                    self.xo = Default::default();
                    changed = true;
                }
            }
            ui.label(egui::RichText::new("Textbook values on a resistive load: a starting point, then tune by ear… and by the charts.").weak());
        });
        ui.toggle_value(&mut self.opt_open, "Optimizer…");
        changed |= self.variants_bar(ui);
        section(ui, "Schematic");
        changed |= self.xo.ui(ui, &mut self.design.schematic, &mut self.design.units);
        changed
    }
    /// Variant memories: recall (Ctrl+click = delete), save a copy, and which others are drawn in the SPL chart.
    fn variants_bar(&mut self, ui: &mut egui::Ui) -> bool {
        let mut changed = false;
        let exists = |d: &Design, k: usize| d.variants.slots.get(k).is_some_and(|s| s.is_some()) || d.variants.active == k;
        ui.horizontal(|ui| {
            ui.label("Variant");
            for k in 0..VARIANT_COUNT {
                let (active, ex) = (self.design.variants.active == k, exists(&self.design, k));
                let r = ui.add_enabled(ex, egui::SelectableLabel::new(active, (k + 1).to_string())).on_hover_text("click: show · Ctrl+click: delete");
                if r.clicked() && !active {
                    if ui.input(|i| i.modifiers.ctrl) {
                        self.variant_confirm = Some((true, k));
                    } else if self.design.recall_variant(k) {
                        self.xo = Default::default();
                        self.opt_undo.clear();
                        changed = true;
                    }
                }
            }
            egui::ComboBox::from_id_salt("variant_save").selected_text("Save copy to…").show_ui(ui, |ui| {
                for k in 0..VARIANT_COUNT {
                    if self.design.variants.active != k && ui.selectable_label(false, format!("#{}{}", k + 1, if exists(&self.design, k) { " (overwrite)" } else { "" })).clicked() {
                        if exists(&self.design, k) {
                            self.variant_confirm = Some((false, k));
                        } else {
                            self.design.save_variant(k);
                            changed = true;
                        }
                    }
                }
            });
        });
        if let Some((delete, k)) = self.variant_confirm {
            ui.horizontal(|ui| {
                ui.label(if delete { format!("Delete crossover variant {}?", k + 1) } else { format!("Overwrite crossover variant {}?", k + 1) });
                if ui.button("Yes").clicked() {
                    if delete {
                        self.design.delete_variant(k);
                        self.variant_show[k] = false;
                    } else {
                        self.design.save_variant(k);
                    }
                    self.variant_confirm = None;
                    changed = true;
                }
                if ui.button("No").clicked() {
                    self.variant_confirm = None;
                }
            });
        }
        let others: Vec<usize> = (0..VARIANT_COUNT).filter(|&k| self.design.variants.active != k && exists(&self.design, k)).collect();
        if !others.is_empty() {
            ui.horizontal(|ui| {
                ui.label("Show in SPL chart:");
                for k in others {
                    changed |= ui.checkbox(&mut self.variant_show[k], (k + 1).to_string()).changed();
                }
            });
        }
        changed
    }

    /// Target, options and the Optimize / Undo buttons (floating window); the parts to adjust are flagged in the schematic.
    fn optimizer_body(&mut self, ui: &mut egui::Ui) -> bool {
        let mut changed = optimizer_ui::settings_ui(ui, &mut self.design.optimizer, &mut self.opt_status);
        ui.checkbox(&mut self.show_target, "Show target in the SPL chart");
        let n = self.design.schematic.parts.iter().filter(|c| c.opt && c.has_value()).count();
        ui.horizontal(|ui| {
            if ui.button("All R, L, C").clicked() {
                self.design.schematic.parts.iter_mut().filter(|c| c.has_value()).for_each(|c| c.opt = true);
                changed = true;
            }
            if ui.button("None").clicked() {
                self.design.schematic.parts.iter_mut().for_each(|c| c.opt = false);
                changed = true;
            }
            ui.label(format!("{n} components selected (also in the part properties)"));
        });
        ui.horizontal(|ui| {
            let busy = self.opt_rx.is_some();
            if ui.add_enabled(!busy && n > 0, egui::Button::new("Optimize")).clicked() {
                self.start_optimizer(ui.ctx());
            }
            if ui.add_enabled(!self.opt_undo.is_empty(), egui::Button::new("Undo")).clicked() {
                for (k, v) in self.opt_undo.pop().unwrap_or_default() {
                    if let Some(c) = self.design.schematic.parts.get_mut(k) {
                        c.value = v;
                    }
                }
                changed = true;
            }
            if busy {
                ui.spinner();
            }
            ui.label(egui::RichText::new(&self.opt_status).weak());
        });
        changed
    }

    fn start_optimizer(&mut self, ctx: &egui::Context) {
        let chosen: Vec<_> = self.design.schematic.parts.iter().enumerate().filter(|(_, c)| c.opt && c.has_value()).collect();
        let limits: Vec<optimize::Param> = chosen.iter().map(|&(i, c)| optimize::Param { index: i, min: Some(c.opt_min), max: Some(c.opt_max) }).collect();
        let parts: Vec<usize> = chosen.iter().map(|&(i, _)| i).collect();
        let (speaker, settings) = (self.design.speaker(), self.design.optimizer.clone());
        let (tx, rx) = channel();
        let ctx = ctx.clone();
        std::thread::spawn(move || {
            let run = || optimize::Optimizer::new(&speaker, &limits, &settings).map(|mut o| (parts.clone(), o.run()));
            let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(run)).unwrap_or_else(|_| Err("internal error in the optimizer".into()));
            let _ = tx.send(r);
            ctx.request_repaint();
        });
        self.opt_rx = Some(rx);
        self.opt_status = "optimizing…".into();
    }

    /// Applies a finished optimizer run to the schematic.
    fn poll_optimizer(&mut self) {
        let Some(rx) = &self.opt_rx else { return };
        let Ok(r) = rx.try_recv() else { return };
        self.opt_rx = None;
        match r {
            Ok((parts, out)) => {
                let old = parts.iter().filter_map(|&k| self.design.schematic.parts.get(k).map(|c| (k, c.value))).collect();
                self.opt_undo.push(old);
                self.opt_undo.truncate(20);
                for (&k, &v) in parts.iter().zip(&out.values) {
                    if let Some(c) = self.design.schematic.parts.get_mut(k) {
                        c.value = v;
                    }
                }
                self.opt_status = format!("mean squared error {:.2} to {:.2} dB2 ({} evaluations)", out.error_before, out.error_after, out.evals);
                self.dirty = true;
            }
            Err(e) => self.opt_status = e,
        }
    }

    /// A bass-reflex box (T/S model or box correction) needs a vent on the front; other drivers have none.
    /// A rectangular vent sets the round vent of equal area used by the box model.
    fn sync_ports(&mut self) {
        for u in &mut self.design.units {
            // a new vent goes below the driver
            let (x, y) = u.port.map_or((u.x, (u.y - u.dd).max(20.0)), |p| (p.0, p.1));
            let r = match &mut u.source {
                Source::Simulated { radiator, .. } | Source::HalfSpace { box_correction: Some((_, radiator)), .. } => Some(radiator),
                _ => None,
            };
            (u.port, u.port_rect) = match r {
                Some(Radiator::BassReflex { diam, .. }) => {
                    if let Some((w, h)) = u.port_rect {
                        *diam = area_diameter(w * h / 100.0) / 10.0;
                    }
                    (Some((x, y, *diam * 10.0)), u.port_rect)
                }
                // a passive radiator sits on the front like a vent: round, of the area of its n diaphragms
                Some(Radiator::Passive { pr, .. }) => (Some((x, y, area_diameter(pr.sd * pr.n as f64))), None),
                _ => (None, None),
            };
        }
    }

    /// The selected driver as a catalog record: T/S (also of the box correction), diameter, responses; the type is
    /// a guess from the diameter (edit it in the catalog). A record with the same maker, model and type is replaced.
    fn save_unit_to_catalog(&mut self) {
        let Some(u) = self.design.units.get(self.selected) else { return };
        let (driver, response, impedance) = match &u.source {
            Source::Simulated { driver, .. } => (Some(driver.clone()), None, None),
            Source::HalfSpace { response, impedance, box_correction } => (box_correction.as_ref().map(|(d, _)| d.clone()), Some(response.clone()), impedance.clone()),
            Source::InCabinet { .. } => {
                self.status = "A driver measured in the cabinet cannot go to the catalog (no half-space response)".into();
                return;
            }
        };
        // the record that came with the unit (maker, type, Xmax, notes, off-axis curves), else a new one
        let had_record = u.record.is_some();
        let mut e = u.record.clone().unwrap_or_else(|| Entry::new(if u.dd < TWEETER_DD { Kind::Tweeter } else { Kind::Woofer }));
        e.dd = u.dd;
        match driver {
            Some(d) => e.driver = d,
            None if !had_record => e.driver.model = u.name.clone(),
            None => {}
        }
        e.response = response.filter(|r| !r.is_empty());
        e.impedance = impedance;
        // without a record, what a unit does not hold stays in the catalog record it replaces
        if let (false, Some(old)) = (had_record, self.catalog.cat.entries.iter().find(|x| x.same_record(&e))) {
            (e.outer_d, e.xmax, e.notes, e.off_axis) = (old.outer_d, old.xmax, old.notes.clone(), old.off_axis.clone());
        }
        if u.outer_d > 0.0 {
            e.outer_d = u.outer_d;
        }
        let replaced = self.catalog.upsert(e.clone());
        self.status = format!("{} {} the catalog", e.title(), if replaced { "updated in" } else { "added to" });
    }

    fn add_catalog_unit(&mut self, e: &Entry) {
        self.add_unit(e.unit_name(), e.source(DEFAULT_BOX), e.dd, e.outer_d);
        if let Some(u) = self.design.units.last_mut() {
            u.record = Some(e.record());
        }
        self.dirty = true;
    }

    /// Adds a unit (name made unique) on a free spot of the front, with a direct-wired way; returns true.
    fn add_unit(&mut self, base: String, source: Source, dd: f64, outer_d: f64) -> bool {
        let name = self.design.unique_name(&base, None);
        let ((x, y), grew) = self.design.make_room(dd, outer_d);
        if grew {
            let (w, h) = self.design.front.size();
            self.status = format!("Front enlarged to {w:.0} × {h:.0} mm to fit {name}");
        }
        self.design.units.push(Unit { outer_d, ..Unit::new(name.clone(), (x, y), dd, source) });
        let way = Way { driver: name, stages: vec![], inverted: false };
        self.design.schematic.replace_branch(&way);
        self.design.ways.push(way);
        self.selected = self.design.units.len() - 1;
        true
    }

    /// Name, placement, source data and box of the selected unit.
    fn unit_panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut changed = false;
        let u = &mut self.design.units[self.selected];
        let old_name = u.name.clone();
        ui.horizontal(|ui| {
            ui.label("Name");
            if ui.text_edit_singleline(&mut u.name).lost_focus() && u.name != old_name {
                changed = true;
            }
        });
        if u.name != old_name {
            let new_name = u.name.clone();
            self.design.relink_driver(&old_name, &new_name);
        }
        let u = &mut self.design.units[self.selected];
        egui::Grid::new("unit").num_columns(2).show(ui, |ui| {
            for (label, v) in [("Diameter Dd", &mut u.dd), ("Outer diameter (frame), 0 = unknown", &mut u.outer_d)] {
                ui.label(label);
                changed |= ui.add(egui::DragValue::new(v).suffix(" mm")).changed();
                ui.end_row();
            }
        });

        // response source
        let mut kind = match u.source {
            Source::Simulated { .. } => 0,
            Source::HalfSpace { .. } => 1,
            Source::InCabinet { .. } => 2,
        };
        let before = kind;
        egui::ComboBox::from_id_salt("source").selected_text(["T/S model", "Half-space response file", "Measured in cabinet"][kind]).show_ui(ui, |ui| {
            ui.selectable_value(&mut kind, 0, "T/S model");
            ui.selectable_value(&mut kind, 1, "Half-space response file");
        });
        if kind != before {
            let ts = match &u.source {
                Source::HalfSpace { box_correction: Some((d, r)), .. } | Source::Simulated { driver: d, radiator: r } => (d.clone(), *r),
                _ => (default_ts(), DEFAULT_BOX),
            };
            u.source = match kind {
                0 => Source::Simulated { driver: ts.0, radiator: ts.1 },
                _ => Source::HalfSpace { response: Response::new("", vec![], vec![]), impedance: None, box_correction: None },
            };
            changed = true;
        }

        let mut status = None;
        match &mut u.source {
            Source::Simulated { driver, .. } => {
                changed |= ts_editor(ui, driver, &mut status);
            }
            Source::HalfSpace { response, impedance, box_correction } => {
                ui.label(if response.freq.is_empty() { "no response loaded".to_string() } else { format!("response: {} ({} points)", response.name, response.len()) });
                ui.label(format!("impedance: {}", impedance.as_ref().map_or("—".to_string(), |z| z.name.clone())));
                ui.horizontal(|ui| {
                    if let Some(r) = load_button(ui, "Load response…", MagUnit::Db, &mut status) {
                        *response = r;
                        changed = true;
                    }
                    if let Some(z) = load_button(ui, "Load impedance…", MagUnit::Linear, &mut status) {
                        *impedance = Some(z);
                        changed = true;
                    }
                });
                let mut bc = box_correction.is_some();
                if ui.checkbox(&mut bc, "Box correction from T/S (response measured on infinite/IEC baffle)").changed() {
                    *box_correction = bc.then(|| (default_ts(), DEFAULT_BOX));
                    changed = true;
                }
                if let Some((driver, _)) = box_correction {
                    changed |= ts_editor(ui, driver, &mut status);
                }
            }
            Source::InCabinet { responses } => {
                ui.label(format!("measured in the cabinet: {}", soundcad_speaker::polar::summary(responses)));
                ui.label(format!("impedance: {}", responses.impedance.as_ref().map_or("—".to_string(), |z| z.name.clone())));
                ui.horizontal(|ui| {
                    if ui.button("Replace polar files…").clicked() {
                        match pick_polars() {
                            Some(Ok((_, mut new))) => {
                                new.impedance = responses.impedance.take();
                                *responses = new;
                                changed = true;
                            }
                            Some(Err(e)) => status = Some(e),
                            None => {}
                        }
                    }
                    if let Some(z) = load_button(ui, "Load impedance…", MagUnit::Linear, &mut status) {
                        responses.impedance = Some(z);
                        changed = true;
                    }
                });
            }
        }
        if let Some(s) = status {
            self.status = s;
        }
        changed
    }
}

/// T/S parameters, with import from a .vxd driver database; returns true when edited.
fn ts_editor(ui: &mut egui::Ui, d: &mut Driver, status: &mut Option<String>) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label(format!("T/S: {} {}", d.manufacturer, d.model));
        if ui.button("Import .vxd…").clicked() {
            if let Some(p) = rfd::FileDialog::new().add_filter("Driver database (.vxd)", &["vxd", "xml"]).pick_file() {
                match std::fs::read_to_string(&p).map(|x| Driver::from_vxd(&x)) {
                    Ok(list) if !list.is_empty() => {
                        *d = list[0].clone();
                        changed = true;
                    }
                    Ok(_) => *status = Some(format!("{}: no driver found", p.display())),
                    Err(e) => *status = Some(format!("{}: {e}", p.display())),
                }
            }
        }
    });
    egui::CollapsingHeader::new("T/S parameters").id_salt("ts").show(ui, |ui| changed |= ts_grid(ui, "ts", d, false));
    changed
}

/// T/S parameters in a grid of two per row (a passive radiator has no motor: Mms, Cms, Rms and Sd only); true when edited.
fn ts_grid(ui: &mut egui::Ui, id: &str, d: &mut Driver, passive: bool) -> bool {
    let mut changed = false;
    egui::Grid::new(id).num_columns(4).show(ui, |ui| {
        let mut rows: Vec<(&str, &mut f64, &str)> = vec![];
        if !passive {
            rows.extend([("Re", &mut d.re, " Ω"), ("Le", &mut d.le, " mH"), ("Leb", &mut d.leb, " mH"), ("Ke", &mut d.ke, " sH"), ("Rss", &mut d.rss, " Ω")]);
        }
        rows.extend([("Rms", &mut d.rms, " Ns/m"), ("Mms", &mut d.mms, " g"), ("Cms", &mut d.cms, " mm/N"), ("Sd", &mut d.sd, " cm²")]);
        if !passive {
            rows.push(("Bl", &mut d.bl, " Tm"));
        }
        for (i, (label, v, suffix)) in rows.into_iter().enumerate() {
            ui.label(label);
            changed |= ui.add(egui::DragValue::new(v).speed(0.01).range(0.0..=1e6).suffix(suffix)).changed();
            if i % 2 == 1 {
                ui.end_row();
            }
        }
    });
    changed
}

/// Radiator types, in the usual order; the ones without a model here are listed but disabled.
const BOX_TYPES: [&str; 9] = ["Infinite baffle", "Closed", "Bass reflex", "Double tuned bass reflex", "Passive radiator", "Band pass type 1", "Band pass type 1P", "Band pass type 2", "Band pass type 3"];

/// Index of the radiator in `BOX_TYPES`.
fn box_kind(r: &Radiator) -> usize {
    match r {
        Radiator::InfiniteBaffle => 0,
        Radiator::Closed { .. } => 1,
        Radiator::BassReflex { .. } => 2,
        Radiator::Passive { .. } => 4,
    }
}

/// Box type and parameters; returns true when edited.
fn box_editor(ui: &mut egui::Ui, r: &mut Radiator, locked: bool) -> bool {
    let mut changed = false;
    let kind = box_kind(r);
    let mut k = kind;
    egui::ComboBox::from_id_salt("box").selected_text(BOX_TYPES[kind]).show_ui(ui, |ui| {
        for (i, n) in BOX_TYPES.iter().enumerate() {
            if matches!(i, 0 | 1 | 2 | 4) {
                ui.selectable_value(&mut k, i, *n);
            } else {
                ui.add_enabled(false, egui::SelectableLabel::new(false, format!("{n} (not available yet)")));
            }
        }
    });
    if k != kind {
        let (vb, qa, ql) = match *r {
            Radiator::Closed { vb, qa, ql } | Radiator::BassReflex { vb, qa, ql, .. } | Radiator::Passive { vb, qa, ql, .. } => (vb, qa, ql),
            Radiator::InfiniteBaffle => (8.0, 5.0, 10.0),
        };
        *r = match k {
            0 => Radiator::InfiniteBaffle,
            1 => Radiator::Closed { vb, qa, ql },
            2 => Radiator::BassReflex { vb, qa, ql, diam: 4.0, len: 15.0, qp: 100.0, n: 1, end_corr: 0.85 },
            _ => {
                let mut pr = PassiveRadiator { vas: 30.0, sd: 100.0, qms: 10.0, fs: 0.0, mms: 40.0, extra: 0.0, n: 1 };
                pr.fs = pr.natural_fs();
                Radiator::Passive { vb, qa, ql, pr }
            }
        };
        changed = true;
    }
    egui::Grid::new("box").num_columns(2).show(ui, |ui| {
        let mut row = |ui: &mut egui::Ui, label: &str, v: &mut f64, suffix: &str| {
            ui.label(label);
            let derived = locked && matches!(label, "Volume" | "Qa");
            changed |= ui.add_enabled(!derived, egui::DragValue::new(v).speed(0.1).range(0.1..=1000.0).suffix(suffix)).changed();
            ui.end_row();
        };
        match r {
            Radiator::InfiniteBaffle => {}
            Radiator::Closed { vb, qa, ql } | Radiator::Passive { vb, qa, ql, .. } => {
                row(ui, "Volume", vb, " l");
                row(ui, "Qa", qa, "");
                row(ui, "Ql", ql, "");
            }
            Radiator::BassReflex { vb, qa, ql, len, qp, end_corr, .. } => {
                row(ui, "Volume", vb, " l");
                row(ui, "Qa", qa, "");
                row(ui, "Ql", ql, "");
                row(ui, "Vent length", len, " cm");
                row(ui, "Vent Qp", qp, "");
                ui.label("End correction");
                changed |= ui.add(egui::DragValue::new(end_corr).speed(0.01).range(0.6..=2.4)).changed();
                ui.end_row();
            }
        }
    });
    if let Radiator::Passive { pr, .. } = r {
        changed |= pr_editor(ui, pr);
    }
    changed
}
/// Cabinet interior: depth, walls and lining, with what follows from them (volume, Qa, absorption, reverberation,
/// internal resonances and how much of them the vent lets out). Returns true when edited.
fn cabinet_editor(ui: &mut egui::Ui, cab: &mut Option<Cabinet>, a: Option<&Analysis>) -> bool {
    let mut changed = false;
    section(ui, "Cabinet interior and damping");
    let mut on = cab.is_some();
    if ui.checkbox(&mut on, "Volume, Qa and internal resonances from the cabinet").changed() {
        *cab = on.then(Cabinet::default);
        changed = true;
    }
    let (Some(c), Some(a)) = (cab.as_mut(), a) else {
        ui.label(egui::RichText::new("Off: the box is an ideal volume with the Qa typed above.").weak());
        return changed;
    };
    let max_pct = if a.surface > 0.0 { a.max_lined / a.surface * 100.0 } else { 0.0 };
    egui::Grid::new("cabinet").num_columns(2).show(ui, |ui| {
        let mut row = |ui: &mut egui::Ui, label: &str, v: &mut f64, range: std::ops::RangeInclusive<f64>, suffix: &str| {
            ui.label(label);
            changed |= ui.add(egui::DragValue::new(v).speed(0.5).range(range).suffix(suffix)).changed();
            ui.end_row();
        };
        row(ui, "Internal depth", &mut c.depth, 20.0..=2000.0, " mm");
        row(ui, "Wall thickness", &mut c.wall, 0.0..=60.0, " mm");
        row(ui, "Lining thickness", &mut c.lining, 0.0..=200.0, " mm");
        row(ui, "Lining density", &mut c.density, 5.0..=300.0, " kg/m³");
        row(ui, "Lined surface", &mut c.coverage, 0.0..=max_pct, " %");
    });
    let (w, h, d) = a.dims;
    ui.label(format!("Inside {:.0} × {:.0} × {:.0} mm = {:.1} l, minus lining {:.1} l and vents {:.1} l: {:.1} l of air, Qa {:.1}", w * 1e3, h * 1e3, d * 1e3, a.gross_l, a.lining_l, a.vents_l, a.vb, a.qa));
    ui.label(format!("Surface {:.3} m², holes {:.3} m²: at most {:.3} m² ({max_pct:.0} %) can be lined; lined {:.3} m². Lining σ ≈ {:.0} Pa·s/m²", a.surface, a.holes, a.max_lined, a.lined, a.sigma));
    egui::Grid::new("cab_abs").num_columns(6).striped(true).show(ui, |ui| {
        let fs = [125.0, 250.0, 500.0, 1000.0, 2000.0];
        ui.label("Frequency");
        for f in fs {
            ui.label(format!("{f:.0} Hz"));
        }
        ui.end_row();
        ui.label("Absorption");
        for f in fs {
            ui.label(format!("{:.2}", a.alpha(f)));
        }
        ui.end_row();
        ui.label("Reverb. time");
        for f in fs {
            ui.label(format!("{:.0} ms", a.t60(f) * 1e3));
        }
        ui.end_row();
    });
    if a.modes.iter().all(|m| m.at_vent == 0.0) {
        ui.label(egui::RichText::new("No vent: internal resonances stay inside the box.").weak());
        return changed;
    }
    ui.add_space(4.0);
    ui.label(egui::RichText::new("Internal resonances heard at the vent (lowest with real pressure at the vent)").strong());
    for m in a.modes.iter().filter(|m| m.at_vent.abs() > 0.2).take(8) {
        ui.label(format!("{:6.0} Hz   mode {}-{}-{}   pressure at vent {:+.0} %   Q {:.0}", m.f, m.n.0, m.n.1, m.n.2, m.at_vent * 100.0, m.q));
    }
    let pts: PlotPoints = (0..240)
        .map(|i| {
            let f = 30.0 * (MODES_MAX_HZ / 30.0).powf(i as f64 / 239.0);
            [f.log10(), 20.0 * a.vent_factor(f).norm().log10()]
        })
        .collect();
    Plot::new("cab_vent")
        .height(150.0)
        .allow_zoom(false)
        .allow_scroll(false)
        .allow_drag(false)
        .x_axis_formatter(|m, _| format!("{:.0} Hz", 10f64.powf(m.value)))
        .y_axis_label("dB vs. uniform pressure")
        .show(ui, |pu| pu.line(Line::new(pts)));
    ui.label(egui::RichText::new("Rectangular box, driver and vent on the front. The cone does not feel the modes; estimate, not a measurement.").weak());
    changed
}
/// Passive radiator parameters as in the Enclosure tool; fs follows Vas, Sd and Mms until edited by hand.
fn pr_editor(ui: &mut egui::Ui, pr: &mut PassiveRadiator) -> bool {
    let (mut changed, mut physical) = (false, false);
    ui.add_space(6.0);
    ui.label(egui::RichText::new("Passive radiator").strong());
    egui::Grid::new("pr").num_columns(2).show(ui, |ui| {
        for (label, v, suffix, phys) in [("Vas", &mut pr.vas, " l", true), ("Sd", &mut pr.sd, " cm²", true), ("Mms", &mut pr.mms, " g", true), ("Qms", &mut pr.qms, "", false), ("Extra mass", &mut pr.extra, " g", false), ("fs", &mut pr.fs, " Hz", false)] {
            ui.label(label);
            let r = ui.add(egui::DragValue::new(v).speed(0.1).range(0.0..=5000.0).suffix(suffix)).changed();
            changed |= r;
            physical |= r && phys;
            ui.end_row();
        }
        ui.label("Number");
        changed |= ui.add(egui::DragValue::new(&mut pr.n).range(1..=24)).changed();
        ui.end_row();
    });
    if physical {
        pr.fs = pr.natural_fs();
    }
    changed
}

/// Curves of chart `k` smoothed to 1/`n` octave; the result is kept in `cache` until the result, chart or setting changes.
fn smoothed_curves(cache: &mut std::collections::HashMap<(u64, u32, usize), Vec<(String, Vec<f64>)>>, n: u32, shown: u64, k: usize, freq: &[f64], curves: &[(String, Vec<f64>)]) -> Vec<(String, Vec<f64>)> {
    let key = (shown, n, k);
    if let Some(v) = cache.get(&key) {
        return v.clone();
    }
    if cache.len() > 16 {
        cache.clear();
    }
    let out: Vec<_> = curves
        .iter()
        .map(|(name, db)| {
            let r = Response::from_db_deg(name.clone(), freq.to_vec(), db, &vec![0.0; db.len()]);
            (name.clone(), soundcad_core::smooth::smooth(&r, soundcad_core::smooth::Smoothing::Oct(n)).db())
        })
        .collect();
    cache.insert(key, out.clone());
    out
}

/// Colours of the curves by position, so "Total" etc. look the same on every chart.
const PALETTE: [Color32; 6] = [
    Color32::from_rgb(230, 80, 70),
    Color32::from_rgb(80, 150, 240),
    Color32::from_rgb(150, 210, 70),
    Color32::from_rgb(240, 170, 50),
    Color32::from_rgb(190, 110, 230),
    Color32::from_rgb(80, 210, 200),
];

/// Legend line height; the legend is drawn under the plot, wrapped to the width, so the plot keeps the space.
const LEGEND_LINE: f32 = 15.0;

/// Vertical grid of a log-frequency axis (x = log10 f): lines at 1…9 × 10^d, and at 1.1…9.9 × 10^d when less than a decade is in view.
/// `step_size` sets how strong a line is and when its label appears (`freq_axis`): decades first, then 2 and 5,
/// the others by the true distance to the next line, so their labels cannot overlap.
fn log_grid(input: egui_plot::GridInput) -> Vec<egui_plot::GridMark> {
    let (lo, hi) = input.bounds;
    let by = if hi - lo < 1.0 { 1 } else { 10 };
    let mut marks = vec![];
    // 1 Hz…10 MHz is enough: a far zoom-out must not generate millions of lines
    for d in lo.floor().max(0.0) as i32..=hi.ceil().min(6.0) as i32 {
        for m in (10..100).step_by(by) {
            let value = d as f64 + (m as f64 / 10.0).log10();
            let step_size = match m {
                10 => 1.0,
                20 | 50 => 0.2,
                m if m % 10 == 0 => ((m + 10) as f64 / m as f64).log10(),
                m => ((m + 1) as f64 / m as f64).log10(),
            };
            if value >= lo - 1e-9 && value <= hi + 1e-9 {
                marks.push(egui_plot::GridMark { value, step_size });
            }
        }
    }
    marks
}

/// Axis label of a frequency: 20, 500, 1.1k, 20k.
fn hz_label(f: f64) -> String {
    let (v, k) = if f >= 1000.0 { (f / 1000.0, "k") } else { (f, "") };
    let s = format!("{v:.1}");
    format!("{}{k}", s.trim_end_matches(".0"))
}

/// Room around the plot area inside a chart cell: value labels on the left, frequency labels below it, then the legend.
/// Fixed and the same for curves and map, so every plot area has the same size. (egui_plot's own axes depend on the label
/// text and also keep room for an empty axis title, about 20 px on each axis: they are off and `axis_labels` draws the labels.)
const AXIS_W: f32 = 28.0;
const AXIS_H: f32 = 15.0;
const CELL_GAP: f32 = 4.0;
const AXIS_FONT: f32 = 11.0;
/// Grid lines closer than this are not drawn (`Plot::grid_spacing`).
const GRID_MIN: f32 = 4.0;

fn plot_area(cell: egui::Rect, legend_h: f32) -> egui::Rect {
    egui::Rect::from_min_max(cell.min + Vec2::new(AXIS_W, 0.0), cell.max - Vec2::new(CELL_GAP, AXIS_H + legend_h))
}

/// Frequency labels under `plot` (x = log10 f from `lo` to `hi`), for every grid line with room for one. Returns their x.
fn freq_labels(p: &egui::Painter, plot: egui::Rect, lo: f64, hi: f64) -> Vec<f32> {
    let per_unit = plot.width() as f64 / (hi - lo);
    let marks = log_grid(egui_plot::GridInput { bounds: (lo, hi), base_step_size: 0.0 });
    marks
        .iter()
        .filter(|m| m.step_size * per_unit >= 30.0)
        .map(|m| {
            let x = plot.left() + ((m.value - lo) * per_unit) as f32;
            p.text(Pos2::new(x, plot.bottom() + 1.0), Align2::CENTER_TOP, hz_label(10f64.powf(m.value)), FontId::proportional(AXIS_FONT), Color32::GRAY);
            x
        })
        .collect()
}

/// Horizontal grid: main lines at the smallest 1-2-5 × 10^n step at least 40 px apart, so every chart has the same density
/// whatever its range; five fine lines per step.
fn value_grid(input: egui_plot::GridInput) -> Vec<egui_plot::GridMark> {
    let (lo, hi) = input.bounds;
    // base_step_size = the value of GRID_MIN pixels
    let wanted = input.base_step_size * 40.0 / GRID_MIN as f64;
    if wanted <= 0.0 || !wanted.is_finite() {
        return vec![];
    }
    let decade = 10f64.powf(wanted.log10().floor());
    let step = [1.0, 2.0, 5.0, 10.0].iter().map(|m| m * decade).find(|s| *s >= wanted).unwrap_or(10.0 * decade);
    let fine = step / 5.0;
    let (first, last) = ((lo / fine).ceil() as i64, (hi / fine).floor() as i64);
    (first..=last).map(|i| egui_plot::GridMark { value: i as f64 * fine, step_size: if i % 5 == 0 { step } else { fine } }).collect()
}

/// Frequency and value labels of a curve plot.
fn axis_labels(p: &egui::Painter, tr: &egui_plot::PlotTransform) {
    let (plot, b) = (*tr.frame(), tr.bounds());
    freq_labels(p, plot, b.min()[0], b.max()[0]);
    let per_unit = tr.dpos_dvalue_y().abs();
    // the same marks as the horizontal grid lines; only the main ones have a label (at least 40 px apart, the fine ones under 20)
    let marks = value_grid(egui_plot::GridInput { bounds: (b.min()[1], b.max()[1]), base_step_size: GRID_MIN as f64 / per_unit });
    for m in marks.iter().filter(|m| m.step_size * per_unit >= 25.0) {
        let y = tr.position_from_point_y(m.value);
        if plot.y_range().contains(y) {
            let decimals = (-m.step_size.log10()).ceil().max(0.0) as usize;
            // + 0.0: no "-0"
            p.text(Pos2::new(plot.left() - 3.0, y), Align2::RIGHT_CENTER, format!("{:.*}", decimals, m.value + 0.0), FontId::proportional(AXIS_FONT), Color32::GRAY);
        }
    }
}

/// Legend rows: greedy wrap of "● name" items to `width`, as (curve index, item width).
fn legend_rows(ui: &egui::Ui, names: &[&str], width: f32) -> Vec<Vec<(usize, f32)>> {
    let mut rows = vec![vec![]];
    let mut used = 0.0;
    for (k, n) in names.iter().enumerate() {
        let w = ui.fonts(|f| f.layout_no_wrap(n.to_string(), FontId::proportional(AXIS_FONT), Color32::WHITE).size().x) + 20.0;
        if used + w > width && !rows.last().unwrap().is_empty() {
            rows.push(vec![]);
            used = 0.0;
        }
        rows.last_mut().unwrap().push((k, w));
        used += w;
    }
    rows
}

/// Chart selector and unit, on the top left corner inside the plot `rect`.
fn chart_selector(ui: &mut egui::Ui, rect: egui::Rect, i: usize, k: &mut usize, note: String, tip: &str) {
    let at = egui::Rect::from_min_size(rect.min + Vec2::splat(4.0), Vec2::new(rect.width() - 8.0, 24.0));
    ui.allocate_new_ui(egui::UiBuilder::new().max_rect(at), |ui| {
        // dark backing: the unit must stay readable over the directivity map and over curves
        egui::Frame::new().fill(Color32::from_black_alpha(190)).corner_radius(3).show(ui, |ui| {
            ui.horizontal(|ui| {
                egui::ComboBox::from_id_salt(("chart", i)).selected_text(CHARTS[*k]).show_ui(ui, |ui| {
                    for (j, name) in CHARTS.iter().enumerate() {
                        ui.selectable_value(k, j, *name);
                    }
                });
                ui.label(note).on_hover_text(tip);
            });
        });
    });
}

/// Log-frequency plot of curves (x = log10 f), 20 Hz…20 kHz, with a compact legend under it, in chart cell `cell`. Returns the plot area.
fn freq_plot(ui: &mut egui::Ui, id: &str, cell: egui::Rect, legend_h: f32, freq: &[f64], curves: &[(String, Vec<f64>)], y: (f64, f64), unit: &str, color0: usize) -> egui::Rect {
    let label_unit = unit.to_string();
    let area = plot_area(cell, legend_h);
    let plot = Plot::new(id)
        .label_formatter(move |name, v| {
            let f = 10f64.powf(v.x);
            let fs = if f >= 1000.0 { format!("{:.2} kHz", f / 1000.0) } else { format!("{f:.0} Hz") };
            if name.is_empty() { format!("{fs}\n{:.1} {label_unit}", v.y) } else { format!("{name}\n{fs}\n{:.1} {label_unit}", v.y) }
        })
        .width(area.width())
        .height(area.height())
        .show_axes(false)
        .set_margin_fraction(Vec2::new(0.0, 0.02))
        .include_x(20f64.log10())
        .include_x(20000f64.log10())
        .include_y(y.0)
        .include_y(y.1)
        .x_grid_spacer(log_grid)
        .y_grid_spacer(value_grid)
        // the lines just below each decade are close together: keep them visible
        .grid_spacing(GRID_MIN..=300.0);
    let tr = ui.allocate_new_ui(egui::UiBuilder::new().max_rect(area), |ui| {
        plot.show(ui, |pu| {
            let wraps = unit == "°";
            for (k, (name, v)) in curves.iter().enumerate() {
                // below the range = cut off (a tweeter at 20 Hz would squash the scale)
                let pts: Vec<[f64; 2]> = freq.iter().zip(v).filter(|(f, d)| (20.0..=20000.0).contains(*f) && **d >= y.0).map(|(f, d)| [f.log10(), *d]).collect();
                // wrapped phase: a new segment at every ±180° jump instead of a vertical line
                let mut segs: Vec<Vec<[f64; 2]>> = vec![vec![]];
                for q in pts {
                    if let Some(last) = segs.last().unwrap().last() {
                        if wraps && (q[1] - last[1]).abs() > 180.0 {
                            segs.push(vec![]);
                        }
                    }
                    segs.last_mut().unwrap().push(q);
                }
                for s in segs {
                    pu.line(Line::new(PlotPoints::from(s)).name(name).color(PALETTE[(k + color0) % PALETTE.len()]));
                }
            }
        })
        .transform
    })
    .inner;
    axis_labels(ui.painter(), &tr);
    let names: Vec<&str> = curves.iter().map(|c| c.0.as_str()).collect();
    let top = area.bottom() + AXIS_H;
    for (r, row) in legend_rows(ui, &names, cell.width()).iter().enumerate() {
        let mut x = cell.left() + 4.0;
        for &(k, w) in row {
            let c = Pos2::new(x + 4.0, top + r as f32 * LEGEND_LINE + LEGEND_LINE / 2.0);
            ui.painter().circle_filled(c, 3.5, PALETTE[(k + color0) % PALETTE.len()]);
            ui.painter().text(c + Vec2::new(8.0, 0.0), Align2::LEFT_CENTER, names[k], FontId::proportional(AXIS_FONT), ui.visuals().text_color());
            x += w;
        }
    }
    area
}
/// `freq_plot` in the normal layout flow, full width (catalog screen).
fn freq_plot_flow(ui: &mut egui::Ui, id: &str, freq: &[f64], curves: &[(String, Vec<f64>)], height: f32, y: (f64, f64), unit: &str, color0: usize) {
    let (cell, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), height), Sense::hover());
    let names: Vec<&str> = curves.iter().map(|c| c.0.as_str()).collect();
    let legend_h = legend_rows(ui, &names, cell.width()).len() as f32 * LEGEND_LINE;
    freq_plot(ui, id, cell, legend_h, freq, curves, y, unit, color0);
    // the plot allocation moved the layout cursor back to the bottom of the plot area
    ui.advance_cursor_after_rect(cell);
}
/// Colour for a level relative to on-axis: 0 dB red … −30 dB dark blue.
fn level_color(db: f64) -> Color32 {
    let t = ((db + 30.0) / 30.0).clamp(0.0, 1.0) as f32;
    let stops = [(0.0, [20, 20, 90]), (0.35, [30, 120, 220]), (0.6, [60, 200, 120]), (0.8, [240, 220, 60]), (1.0, [230, 50, 40])];
    let i = stops.iter().rposition(|s| s.0 <= t).unwrap_or(0).min(stops.len() - 2);
    let (a, b) = (stops[i], stops[i + 1]);
    let k = (t - a.0) / (b.0 - a.0);
    let mix = |j: usize| (a.1[j] as f32 + (b.1[j] as f32 - a.1[j] as f32) * k) as u8;
    Color32::from_rgb(mix(0), mix(1), mix(2))
}

/// Directivity map: x = log frequency 20 Hz…20 kHz, y = angle, colour = level vs on-axis, 3 dB contours by colour bands.
/// The colours are an image built once per result (`cache`: result id + texture) and stretched over the plot:
/// the grid is log-uniform, so equal pixels are equal log-frequency steps. Drawing thousands of cells every
/// frame made the whole window sluggish on software rendering. Returns the plot area.
fn polar_map(ui: &mut egui::Ui, cell: egui::Rect, legend_h: f32, freq: &[f64], polar: &Polar, id: u64, cache: &mut Option<(u64, egui::TextureHandle)>) -> egui::Rect {
    let plot = plot_area(cell, legend_h);
    let resp = ui.allocate_rect(plot, Sense::hover());
    let p = ui.painter();
    let (l0, l1) = (20f64.log10(), 20000f64.log10());
    let x_of = |f: f64| plot.left() + ((f.log10() - l0) / (l1 - l0)) as f32 * plot.width();
    let n = polar.angles.len();
    let row_h = plot.height() / n as f32;
    let cols: Vec<usize> = (0..freq.len()).filter(|&i| (20.0..=20000.0).contains(&freq[i])).collect();
    if cache.as_ref().is_none_or(|(k, _)| *k != id) {
        let mut img = egui::ColorImage::new([cols.len(), n], Color32::BLACK);
        for (ri, row) in polar.rows.iter().enumerate() {
            // +90° at the top; 3 dB bands, like a contour plot: (−3, 0] → 0, (−6, −3] → −3, …
            for (x, &i) in cols.iter().enumerate() {
                img[(x, n - 1 - ri)] = level_color((row[i] / 3.0).ceil() * 3.0);
            }
        }
        *cache = Some((id, ui.ctx().load_texture("directivity", img, egui::TextureOptions::NEAREST)));
    }
    if let (Some((_, tex)), Some(&first), Some(&last)) = (cache.as_ref(), cols.first(), cols.last()) {
        let r = egui::Rect::from_min_max(Pos2::new(x_of(freq[first]), plot.top()), Pos2::new(x_of(freq[last]), plot.bottom()));
        p.image(tex.id(), r, egui::Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
    }
    for x in freq_labels(p, plot, l0, l1) {
        p.line_segment([Pos2::new(x, plot.top()), Pos2::new(x, plot.bottom())], Stroke::new(0.5_f32, Color32::from_black_alpha(90)));
    }
    for a in [-90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0] {
        let y = plot.top() + ((90.0 - a) / 180.0) as f32 * (plot.height() - row_h) + row_h / 2.0;
        p.text(Pos2::new(plot.left() - 3.0, y), Align2::RIGHT_CENTER, format!("{a:+.0}°"), FontId::proportional(AXIS_FONT), Color32::GRAY);
    }
    // hover readout
    if let Some(q) = resp.hover_pos().filter(|q| plot.contains(*q)) {
        let f = 10f64.powf(l0 + ((q.x - plot.left()) / plot.width()) as f64 * (l1 - l0));
        let ri = n - 1 - (((q.y - plot.top()) / row_h) as usize).min(n - 1);
        let i = freq.partition_point(|&x| x < f).min(freq.len() - 1);
        let label = format!("{:.0} Hz, {:+.0}°: {:.1} dB", f, polar.angles[ri], polar.rows[ri][i]);
        let g = p.layout_no_wrap(label, FontId::proportional(13.0), Color32::WHITE);
        let at = q + Vec2::new(14.0, -14.0 - g.size().y);
        p.rect_filled(egui::Rect::from_min_size(at, g.size()).expand(4.0), 3.0, Color32::from_black_alpha(200));
        p.galley(at, g, Color32::WHITE);
    }
    // legend, in the same row as the legend of the curves
    let step = ((cell.width() - 170.0) / 11.0).clamp(6.0, 30.0);
    let lg = egui::Rect::from_min_size(Pos2::new(plot.left(), plot.bottom() + AXIS_H + 2.0), Vec2::new(11.0 * step, LEGEND_LINE - 4.0));
    for k in 0..11 {
        let v = -30.0 + 3.0 * k as f64;
        let r = egui::Rect::from_min_size(lg.min + Vec2::new(k as f32 * step, 0.0), Vec2::new(step, lg.height()));
        p.rect_filled(r, 0.0, level_color(v));
    }
    p.text(lg.right_center() + Vec2::new(8.0, 0.0), Align2::LEFT_CENTER, "−30 … 0 dB (3 dB bands)", FontId::proportional(AXIS_FONT), Color32::GRAY);
    plot
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        while let Ok((id, r)) = self.from_worker.try_recv() {
            if id >= self.shown {
                self.shown = id;
                match r {
                    Ok(r) => {
                        self.results = Some(r);
                        self.error = None;
                    }
                    Err(e) => self.error = Some(e),
                }
            }
        }
        self.poll_optimizer();
        let title = format!("SoundCAD — {}", self.path.as_ref().and_then(|p| p.file_name()).map_or("unsaved".into(), |n| n.to_string_lossy().to_string()));
        // only on change: a title command every frame disturbs mouse drags
        if title != self.title {
            ctx.send_viewport_cmd(egui::ViewportCommand::Title(title.clone()));
            self.title = title;
        }
        egui::TopBottomPanel::top("top").show(ctx, |ui| self.top_bar(ui));
        self.catalog.add_window(ctx);
        self.catalog.trace_window(ctx);
        self.catalog.save_if_dirty(ctx);
        if self.show_catalog {
            let (close, add) = egui::CentralPanel::default().show(ctx, |ui| self.catalog.show(ui)).inner;
            let before = self.design.front.size();
            for e in &add {
                self.add_catalog_unit(e);
            }
            if close || !add.is_empty() {
                self.show_catalog = false;
                self.tab = 0;
            }
            if !add.is_empty() {
                let (w, h) = self.design.front.size();
                let grown = if (w, h) != before { format!(" (front enlarged to {w:.0} × {h:.0} mm)") } else { String::new() };
                self.status = format!("{} added to the project{grown}", add.len());
            }
            return;
        }
        if self.picking || self.replacing.is_some() {
            let old = self.replacing.and_then(|i| self.design.units.get(i)).map(|u| u.name.clone());
            let title = old.as_ref().map_or("Add driver from catalog".to_string(), |n| format!("Replace {n} with"));
            let mut open = true;
            if let Some(e) = self.catalog.picker(ctx, &title, &mut open) {
                match self.replacing {
                    Some(i) if i < self.design.units.len() => {
                        self.design.replace_unit(i, &e);
                        self.status = format!("{} replaced by {}", old.unwrap_or_default(), self.design.units[i].name);
                        self.dirty = true;
                    }
                    _ => self.add_catalog_unit(&e),
                }
            }
            if !open {
                (self.picking, self.replacing) = (false, None);
            }
        }
        if self.opt_open {
            let mut open = true;
            let mut changed = false;
            egui::Window::new("Optimizer").open(&mut open).default_width(430.0).show(ctx, |ui| changed = self.optimizer_body(ui));
            self.opt_open = open;
            self.dirty |= changed;
        }
        let third = ctx.screen_rect().width() / 3.0;
        egui::SidePanel::left("panel").resizable(true).default_width(third).show(ctx, |ui| self.left_panel(ui));
        egui::CentralPanel::default().frame(egui::Frame::central_panel(&ctx.style()).inner_margin(egui::Margin::same(3))).show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label("Smoothing");
                egui::ComboBox::from_id_salt("smoothing").selected_text(if self.smooth == 0 { "none".to_string() } else { format!("1/{} oct", self.smooth) }).show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.smooth, 0, "none");
                    for n in [24, 12, 6, 3] {
                        ui.selectable_value(&mut self.smooth, n, format!("1/{n} oct"));
                    }
                });
                ui.separator();
                match (&self.results, &self.error) {
                    (_, Some(e)) => {
                        ui.colored_label(Color32::RED, e);
                    }
                    (Some(r), None) => {
                        ui.label(format!("computed in {:.0} ms{}", r.seconds * 1000.0, if self.sent > self.shown { " · updating…" } else { "" }));
                    }
                    (None, None) if self.design.units.is_empty() => {
                        ui.label("Add a driver to start (Drivers tab).");
                    }
                    (None, None) => {
                        ui.spinner();
                    }
                }
            });
            let Some(r) = &self.results else { return };
            let (cols, rows) = match self.panels {
                1 | 2 => (1, self.panels),
                4 => (2, 2),
                _ => (2, 3),
            };
            let mut area = ui.available_rect_before_wrap();
            // room for half of the "20k" label at the right end of the last column
            area.max.x -= 8.0;
            let (cw, ch) = (area.width() / cols as f32, area.height() / rows as f32);
            let show_target = self.show_target && !self.design.schematic.parts.is_empty();
            // one legend height for all, the tallest: it is what keeps the plot areas equal
            let names = |k: usize| -> Vec<&str> { if k == MAP { vec![] } else { r.charts[k].curves.iter().map(|c| c.0.as_str()).chain((k == 0 && show_target).then_some("Target")).collect() } };
            let legend_lines = self.panel_chart[..self.panels].iter().map(|&k| legend_rows(ui, &names(k), cw).len()).max().unwrap_or(1);
            let legend_h = legend_lines as f32 * LEGEND_LINE;
            for i in 0..self.panels {
                let cell = egui::Rect::from_min_size(area.min + Vec2::new((i % cols) as f32 * cw, (i / cols) as f32 * ch), Vec2::new(cw, ch));
                let k = &mut self.panel_chart[i];
                ui.allocate_new_ui(egui::UiBuilder::new().max_rect(cell), |ui| {
                    let note = if *k == MAP { "dB vs on-axis".to_string() } else { r.charts[*k].unit.to_string() };
                    let tip = if *k == MAP { "horizontal directivity, dB relative to the listening axis" } else { r.charts[*k].title };
                    // the selector comes after the plot: it is on top and takes the clicks
                    let area = if *k == MAP {
                        polar_map(ui, cell, legend_h, &r.freq, &r.polar, self.shown, &mut self.map_cache)
                    } else {
                        let c = &r.charts[*k];
                        // magnitude charts (SPL, CTA-2034, DI, crossover filters) can be smoothed, globally
                        let shown = self.shown;
                        let smoothed = if self.smooth > 0 && matches!(*k, 0 | 3 | 4 | 7) { Some(smoothed_curves(&mut self.smooth_cache, self.smooth, shown, *k, &r.freq, &c.curves)) } else { None };
                        let base: &[(String, Vec<f64>)] = smoothed.as_deref().unwrap_or(&c.curves);
                        if *k == 0 && show_target {
                            let o = &self.design.optimizer;
                            let mut curves = base.to_vec();
                            curves.push(("Target".into(), r.freq.iter().map(|&f| if f >= o.f_lo && f <= o.f_hi { o.target_db(f) } else { f64::NEG_INFINITY }).collect()));
                            freq_plot(ui, &format!("panel{i}"), cell, legend_h, &r.freq, &curves, c.y, c.unit, c.color0)
                        } else {
                            freq_plot(ui, &format!("panel{i}"), cell, legend_h, &r.freq, base, c.y, c.unit, c.color0)
                        }
                    };
                    chart_selector(ui, area, i, k, note, tip);
                });
            }
        });
        if self.dirty {
            self.submit();
        }
    }
}

fn main() -> eframe::Result {
    use eframe::egui_wgpu::{WgpuConfiguration, WgpuSetup, WgpuSetupCreateNew};
    // any adapter will do, a real GPU first, else the software one (WARP on DX12, e.g. in a VM)
    let pick: eframe::egui_wgpu::NativeAdapterSelectorMethod = std::sync::Arc::new(|adapters: &[eframe::wgpu::Adapter], _| {
        adapters.iter().min_by_key(|a| a.get_info().device_type == eframe::wgpu::DeviceType::Cpu).cloned().ok_or_else(|| "no graphics adapter".to_string())
    });
    let setup = WgpuSetupCreateNew { native_adapter_selector: Some(pick), ..Default::default() };
    let options = eframe::NativeOptions {
        // maximized: a fixed size can run off a 1080p screen, hiding what is at the bottom
        viewport: egui::ViewportBuilder::default().with_maximized(true).with_title("SoundCAD"),
        renderer: eframe::Renderer::Wgpu,
        wgpu_options: WgpuConfiguration { wgpu_setup: WgpuSetup::CreateNew(setup), ..Default::default() },
        ..Default::default()
    };
    eframe::run_native("SoundCAD", options, Box::new(|cc| Ok(Box::new(App::new(cc)))))
}
