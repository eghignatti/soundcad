//! Driver catalog screen: list with filter on the left, the selected record (maker, model, type, T/S,
//! responses) on the right. Edits are written to the catalog file when the mouse is released.

use eframe::egui;
use std::collections::BTreeSet;
use std::path::PathBuf;
use soundcad_core::io::MagUnit;
use soundcad_core::Response;
use soundcad_speaker::catalog::{Catalog, Entry, Kind};
use soundcad_tools::enclosure::area_diameter;

use crate::trace_ui::{Target, TraceUi};

pub struct CatalogUi {
    pub cat: Catalog,
    pub path: PathBuf,
    sel: usize,
    filter: String,
    kind: Option<Kind>,
    dirty: bool,
    pub status: String,
    /// Angle offered for the next "Add SPL".
    angle: f64,
    adding: Option<AddDriver>,
    /// Records ticked for "Add to project" (indices: cleared when records are deleted or the file changes).
    ticked: BTreeSet<usize>,
    confirm_delete: bool,
    /// Graph digitizer, and the record its curve goes to: the Add driver window (None) or a catalog record.
    tracer: TraceUi,
    trace_for: Option<usize>,
}

/// The "Add driver" window: a record not yet in the catalog.
struct AddDriver {
    e: Entry,
    edited: bool,
    confirm_discard: bool,
    confirm_replace: bool,
}

impl CatalogUi {
    pub fn load() -> Self {
        let path = std::env::current_exe().map_or_else(|_| PathBuf::from("catalog.json"), |exe| Catalog::default_path(&exe));
        let (cat, status) = match Catalog::load(&path) {
            Ok(c) => {
                let s = format!("{} drivers", c.entries.len());
                (c, s)
            }
            Err(_) if !path.exists() => (Catalog::default(), "new catalog".into()),
            // moved aside: the next save would overwrite it
            Err(e) => {
                let bad = path.with_extension("json.bad");
                let moved = std::fs::rename(&path, &bad).map_or_else(|m| format!("not moved: {m}"), |_| format!("moved to {}", bad.display()));
                (Catalog::default(), format!("{} unreadable ({moved}): {e}", path.display()))
            }
        };
        CatalogUi { cat, path, sel: 0, filter: String::new(), kind: None, dirty: false, status, angle: 0.0, adding: None, ticked: BTreeSet::new(), confirm_delete: false, tracer: TraceUi::default(), trace_for: None }
    }

    /// The Graph Trace window; a finished curve goes to the record it was opened for.
    pub fn trace_window(&mut self, ctx: &egui::Context) {
        let Some((target, r)) = self.tracer.show(ctx) else { return };
        let entry = match self.trace_for {
            None => self.adding.as_mut().map(|a| {
                a.edited = true;
                &mut a.e
            }),
            Some(i) => {
                self.dirty = true;
                self.cat.entries.get_mut(i)
            }
        };
        let Some(e) = entry else {
            self.status = "The driver of this curve is gone: curve not stored".into();
            return;
        };
        match target {
            Target::Spl(a) => {
                let replaced = e.set_spl(a, r);
                self.angle = (a + 30.0).min(180.0);
                self.status = format!("SPL at {a:.0}° {}", if replaced { "replaced" } else { "added" });
            }
            Target::Impedance => {
                e.impedance = Some(r);
                self.status = "Impedance traced".into();
            }
        }
    }

    /// The "Add driver" window, nothing goes to the catalog before Save.
    pub fn add_window(&mut self, ctx: &egui::Context) {
        let Some(mut a) = self.adding.take() else { return };
        let (mut close, mut save) = (false, false);
        // beside the catalog list, not over it
        egui::Window::new("Add driver").collapsible(false).default_width(480.0).default_pos([340.0, 70.0]).show(ctx, |ui| {
            egui::ScrollArea::vertical().id_salt("addscroll").max_height(ctx.screen_rect().height() - 140.0).show(ui, |ui| {
                let mut trace = None;
                a.edited |= edit(ui, &mut a.e, &mut self.angle, &mut trace, &mut self.status);
                if let Some(t) = trace {
                    self.trace_for = None;
                    self.tracer.open(t);
                }
            });
            ui.separator();
            let exists = self.cat.entries.iter().any(|x| x.same_record(&a.e));
            a.confirm_replace &= exists;
            ui.horizontal(|ui| {
                // the buttons never change place, and "Keep editing" comes first: a double click on Save or
                // Cancel must not land on Replace or Discard
                if a.confirm_replace || a.confirm_discard {
                    ui.label(if a.confirm_replace { "Replace the driver with this name in the catalog?" } else { "Discard this driver?" });
                    if ui.button("Keep editing").clicked() {
                        (a.confirm_replace, a.confirm_discard) = (false, false);
                    }
                    if a.confirm_replace {
                        save = ui.button("Replace").clicked();
                    } else {
                        close = ui.button("Discard").clicked();
                    }
                    return;
                }
                if ui.add_enabled(!a.e.title().is_empty(), egui::Button::new("Save to catalog")).clicked() {
                    save = !exists;
                    a.confirm_replace = exists;
                }
                if ui.button("Cancel").clicked() {
                    close = !a.edited;
                    a.confirm_discard = a.edited;
                }
                if a.e.title().is_empty() {
                    ui.label(egui::RichText::new("Enter maker or model first").weak());
                } else if exists {
                    ui.label(egui::RichText::new("This name is already in the catalog").color(ui.visuals().warn_fg_color));
                }
            });
        });
        if save {
            self.upsert(a.e);
        } else if !close {
            self.adding = Some(a);
        }
    }

    fn matches(&self, e: &Entry) -> bool {
        self.kind.map_or(true, |k| k == e.kind) && e.title().to_lowercase().contains(&self.filter.to_lowercase())
    }

    /// Writes the file after an edit, but not while a drag is in progress.
    pub fn save_if_dirty(&mut self, ctx: &egui::Context) {
        if self.dirty && !ctx.input(|i| i.pointer.any_down()) {
            self.dirty = false;
            self.status = match self.cat.save(&self.path) {
                Ok(()) => format!("{} drivers · saved {}", self.cat.entries.len(), self.path.display()),
                Err(e) => format!("save failed: {e}"),
            };
        }
    }

    /// Filter box + type chips + the list; returns the index of the clicked entry. `ticks`: with "Add to project"
    /// check boxes (a passive radiator is part of a box, it cannot be added as a driver).
    fn list(&mut self, ui: &mut egui::Ui, hide_passive: bool, ticks: bool) -> Option<usize> {
        ui.horizontal(|ui| {
            ui.label("Search");
            ui.text_edit_singleline(&mut self.filter);
        });
        ui.horizontal_wrapped(|ui| {
            ui.selectable_value(&mut self.kind, None, "All");
            for k in Kind::ALL {
                if !(hide_passive && k == Kind::Passive) {
                    ui.selectable_value(&mut self.kind, Some(k), k.label());
                }
            }
        });
        ui.separator();
        let mut clicked = None;
        egui::ScrollArea::vertical().id_salt("catlist").show(ui, |ui| {
            for (i, e) in self.cat.entries.iter().enumerate() {
                if (hide_passive && e.kind == Kind::Passive) || !self.matches(e) {
                    continue;
                }
                let t = e.title();
                let label = format!("{}  ·  {}", if t.is_empty() { "(unnamed)" } else { &t }, e.kind.label());
                ui.horizontal(|ui| {
                    if ticks && e.kind != Kind::Passive {
                        let mut on = self.ticked.contains(&i);
                        if ui.checkbox(&mut on, "").on_hover_text("Tick the drivers to add to the project").changed() {
                            if on {
                                self.ticked.insert(i);
                            } else {
                                self.ticked.remove(&i);
                            }
                        }
                    }
                    if ui.selectable_label(i == self.sel, label).clicked() {
                        clicked = Some(i);
                    }
                });
            }
        });
        clicked
    }

    /// Adds `e`, or replaces the record with the same maker, model and type; returns true when it replaced one.
    pub fn upsert(&mut self, e: Entry) -> bool {
        let same = self.cat.entries.iter().position(|x| x.same_record(&e));
        self.status = format!("{} saved to the catalog", e.title());
        self.dirty = true;
        match same {
            Some(i) => {
                self.cat.entries[i] = e;
                self.sel = i;
                true
            }
            None => {
                self.cat.entries.push(e);
                self.sel = self.cat.entries.len() - 1;
                false
            }
        }
    }

    /// Database menu, the records; true when one was used (the catalog screen should be shown).
    pub fn menu_records(&mut self, ui: &mut egui::Ui) -> bool {
        let mut used = false;
        let n = self.cat.entries.len();
        if ui.button("Add driver…").clicked() {
            self.angle = 0.0;
            let kind = self.kind.unwrap_or(Kind::Woofer);
            self.adding.get_or_insert_with(|| AddDriver { e: Entry::new(kind), edited: false, confirm_discard: false, confirm_replace: false });
            used = true;
        }
        if n > 0 && ui.button("Duplicate driver").clicked() {
            let mut e = self.cat.entries[self.sel.min(n - 1)].clone();
            e.driver.model.push_str(" copy");
            self.cat.entries.push(e);
            self.sel = n;
            self.dirty = true;
            used = true;
        }
        if n > 0 && ui.button("Delete driver…").clicked() {
            self.confirm_delete = true;
            used = true;
        }
        used
    }

    /// Database menu, import and catalog files; true when one was used.
    pub fn menu_files(&mut self, ui: &mut egui::Ui) -> bool {
        let mut used = false;
        if ui.button("Import .vxd driver database…").clicked() {
            used = true;
            if let Some(p) = rfd::FileDialog::new().add_filter("Driver database (.vxd)", &["vxd", "xml"]).pick_file() {
                match std::fs::read_to_string(&p) {
                    Ok(x) => {
                        let n = self.cat.import_vxd(&x);
                        self.status = format!("imported {n} drivers (set type and diameter if needed)");
                        self.dirty |= n > 0;
                    }
                    Err(e) => self.status = format!("{}: {e}", p.display()),
                }
            }
        }
        if ui.button("Open catalog file…").clicked() {
            used = true;
            if let Some(p) = rfd::FileDialog::new().add_filter("Catalog", &["json"]).pick_file() {
                match Catalog::load(&p) {
                    Ok(c) => {
                        self.cat = c;
                        self.path = p;
                        self.sel = 0;
                        self.ticked.clear();
                        if self.trace_for.is_some() {
                            self.tracer.close();
                        }
                        self.status = format!("{} drivers", self.cat.entries.len());
                    }
                    Err(e) => self.status = format!("{}: {e}", p.display()),
                }
            }
        }
        if ui.button("Save catalog file as…").clicked() {
            used = true;
            if let Some(p) = rfd::FileDialog::new().add_filter("Catalog", &["json"]).set_file_name("catalog.json").save_file() {
                self.path = p;
                self.dirty = true;
            }
        }
        used
    }

    /// The whole catalog screen (other actions are in the Database menu). Returns: go back to the project, and
    /// the records to add to it.
    pub fn show(&mut self, ui: &mut egui::Ui) -> (bool, Vec<Entry>) {
        let (mut close, mut add) = (false, vec![]);
        ui.horizontal(|ui| {
            ui.heading("Driver catalog");
            close = ui.button("Back to project").clicked();
            if self.confirm_delete && !self.cat.entries.is_empty() {
                self.sel = self.sel.min(self.cat.entries.len() - 1);
                ui.separator();
                let t = self.cat.entries[self.sel].title();
                ui.label(format!("Delete {} from the catalog?", if t.is_empty() { "(unnamed)" } else { &t }));
                if ui.button("Keep").clicked() {
                    self.confirm_delete = false;
                }
                if ui.button("Delete").clicked() {
                    self.cat.entries.remove(self.sel);
                    self.sel = self.sel.saturating_sub(1);
                    self.ticked.clear();
                    // record indices have shifted
                    if self.trace_for.is_some() {
                        self.tracer.close();
                    }
                    self.dirty = true;
                    self.confirm_delete = false;
                }
            }
        });
        ui.separator();
        egui::SidePanel::left("catalog_list").resizable(true).default_width(300.0).show_inside(ui, |ui| {
            let n = self.ticked.len();
            if ui.add_enabled(n > 0, egui::Button::new(format!("Add to project ({n})"))).on_disabled_hover_text("Tick the drivers to add").clicked() {
                add = self.ticked.iter().filter_map(|&i| self.cat.entries.get(i).cloned()).collect();
                self.ticked.clear();
            }
            ui.separator();
            if let Some(i) = self.list(ui, false, true) {
                self.sel = i;
            }
        });
        if self.cat.entries.is_empty() {
            ui.label("The catalog is empty: Database > Add driver, or import a .vxd driver database.");
            return (close, add);
        }
        self.sel = self.sel.min(self.cat.entries.len() - 1);
        let (mut changed, mut trace) = (false, None);
        egui::ScrollArea::vertical().id_salt("catedit").show(ui, |ui| changed = edit(ui, &mut self.cat.entries[self.sel], &mut self.angle, &mut trace, &mut self.status));
        self.dirty |= changed;
        if let Some(t) = trace {
            self.trace_for = Some(self.sel);
            self.tracer.open(t);
        }
        (close, add)
    }

    /// Pop-up used by the Drivers tab to pick a driver; returns the chosen record.
    pub fn picker(&mut self, ctx: &egui::Context, title: &str, open: &mut bool) -> Option<Entry> {
        let mut out = None;
        let mut win = *open;
        // below the Drivers list, not over it
        egui::Window::new(title).id(egui::Id::new("driver_picker")).open(&mut win).default_width(420.0).default_pos([40.0, 270.0]).show(ctx, |ui| {
            if self.cat.entries.is_empty() {
                ui.label("The catalog is empty: Database > Add driver.");
            }
            ui.set_min_height(300.0);
            if let Some(i) = self.list(ui, true, false) {
                out = Some(self.cat.entries[i].clone());
            }
        });
        *open = win && out.is_none();
        out
    }
}

fn response_row(ui: &mut egui::Ui, label: &str, slot: &mut Option<Response>, unit: MagUnit, status: &mut String) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label(format!("{label}: {}", slot.as_ref().map_or("none".to_string(), |r| format!("{} ({} points)", r.name, r.len()))));
        if ui.button("Load…").clicked() {
            match crate::pick_response(unit) {
                Some(Ok(r)) => {
                    *slot = Some(r);
                    changed = true;
                }
                Some(Err(e)) => *status = e,
                None => {}
            }
        }
        if slot.is_some() && ui.button("Clear").clicked() {
            *slot = None;
            changed = true;
        }
    });
    changed
}

/// SPL curves of a record: list with remove, angle + "Add SPL" from a file or a graph, plot of all of them.
fn spl_curves(ui: &mut egui::Ui, e: &mut Entry, angle: &mut f64, trace: &mut Option<Target>, status: &mut String) -> bool {
    let mut changed = false;
    let mut remove = None;
    for (a, r) in e.spl_curves() {
        ui.horizontal(|ui| {
            ui.label(format!("{a:.0}°: {} ({} points)", r.name, r.len()));
            if ui.small_button("x").on_hover_text("Remove this curve").clicked() {
                remove = Some(a);
            }
        });
    }
    if let Some(a) = remove {
        e.remove_spl(a);
        changed = true;
    }
    ui.horizontal(|ui| {
        ui.label("Angle");
        ui.add(egui::DragValue::new(angle).speed(1.0).range(0.0..=180.0).suffix("°").fixed_decimals(0));
        if ui.button("Trace SPL from image…").on_hover_text("Digitize the curve at this angle from the datasheet graph; a curve already at this angle is replaced").clicked() {
            *trace = Some(Target::Spl(angle.round()));
        }
        if ui.button("Add SPL from file…").on_hover_text("Half-space curve at this angle; a curve already at this angle is replaced").clicked() {
            match crate::pick_response(MagUnit::Db) {
                Some(Ok(r)) => {
                    e.set_spl(angle.round(), r);
                    *angle = (angle.round() + 30.0).min(180.0);
                    changed = true;
                }
                Some(Err(m)) => *status = m,
                None => {}
            }
        }
    });
    let curves = e.spl_curves();
    if let Some((_, first)) = curves.first() {
        let db: Vec<(String, Vec<f64>)> = curves.iter().map(|(a, r)| (format!("{a:.0}°"), r.resample(&first.freq).db())).collect();
        let top = db.iter().flat_map(|(_, v)| v.iter().cloned()).fold(f64::MIN, f64::max);
        super::freq_plot_flow(ui, "cat_spl", &first.freq, &db, 180.0, (top - 40.0, top + 5.0), "dB", 0);
    }
    if !e.off_axis.is_empty() {
        ui.label(egui::RichText::new("The simulation uses the 0° curve; the other angles are only stored for now.").weak());
    }
    changed
}

/// `trace`: set when the user asks to digitize a curve for this record.
fn edit(ui: &mut egui::Ui, e: &mut Entry, angle: &mut f64, trace: &mut Option<Target>, status: &mut String) -> bool {
    let mut changed = false;
    egui::Grid::new("catid").num_columns(2).show(ui, |ui| {
        ui.label("Maker");
        changed |= ui.text_edit_singleline(&mut e.driver.manufacturer).changed();
        ui.end_row();
        ui.label("Model");
        changed |= ui.text_edit_singleline(&mut e.driver.model).changed();
        ui.end_row();
        ui.label("Type");
        egui::ComboBox::from_id_salt("kind").selected_text(e.kind.label()).show_ui(ui, |ui| {
            for k in Kind::ALL {
                changed |= ui.selectable_value(&mut e.kind, k, k.label()).changed();
            }
        });
        ui.end_row();
        ui.label("Effective diameter Dd");
        ui.horizontal(|ui| {
            changed |= ui.add(egui::DragValue::new(&mut e.dd).range(0.0..=1000.0).suffix(" mm")).changed();
            if e.driver.sd > 0.0 {
                let from_sd = area_diameter(e.driver.sd);
                if ui.small_button(format!("{from_sd:.1} mm from Sd")).clicked() {
                    e.dd = (from_sd * 10.0).round() / 10.0;
                    changed = true;
                }
            }
        });
        ui.end_row();
        ui.label("Outer diameter (frame)");
        ui.horizontal(|ui| {
            changed |= ui.add(egui::DragValue::new(&mut e.outer_d).range(0.0..=1000.0).suffix(" mm")).changed();
            if e.outer_d > 0.0 && e.outer_d < e.dd {
                ui.label(egui::RichText::new("smaller than Dd").color(ui.visuals().warn_fg_color));
            } else if e.outer_d == 0.0 {
                ui.label(egui::RichText::new("not set").weak());
            }
        });
        ui.end_row();
        ui.label("Xmax (one way)");
        changed |= ui.add(egui::DragValue::new(&mut e.xmax).speed(0.1).range(0.0..=100.0).suffix(" mm")).changed();
        ui.end_row();
    });
    ui.add_space(4.0);
    let passive = e.kind == Kind::Passive;
    ui.strong("Thiele/Small parameters");
    changed |= super::ts_grid(ui, "catts", &mut e.driver, passive);
    if let Some(x) = e.derived() {
        let q = |v: f64| if v.is_finite() { format!("{v:.2}") } else { "—".into() };
        let text = if passive { format!("Fs {:.1} Hz · Vas {:.1} l", x.fs, x.vas) } else { format!("Fs {:.1} Hz · Qms {} · Qes {} · Qts {} · Vas {:.1} l", x.fs, q(x.qms), q(x.qes), q(x.qts), x.vas) };
        ui.label(egui::RichText::new(text).weak());
    }
    ui.add_space(4.0);
    if passive {
        ui.label(egui::RichText::new("Passive radiators are used from the box section of the Drivers tab.").weak());
    } else {
        ui.strong("Frequency responses (half space)");
        changed |= spl_curves(ui, e, angle, trace, status);
        ui.add_space(4.0);
        changed |= response_row(ui, "Impedance", &mut e.impedance, MagUnit::Linear, status);
        if ui.button("Trace impedance from image…").clicked() {
            *trace = Some(Target::Impedance);
        }
        if let Some(z) = &e.impedance {
            let m: Vec<f64> = z.values.iter().map(|v| v.norm()).collect();
            let top = m.iter().cloned().fold(0.0, f64::max);
            super::freq_plot_flow(ui, "cat_z", &z.freq, &[("|Z|".into(), m)], 140.0, (0.0, top * 1.1), "Ω", 1);
        }
    }
    ui.add_space(4.0);
    ui.label("Notes");
    changed |= ui.add(egui::TextEdit::multiline(&mut e.notes).desired_rows(3).desired_width(f32::INFINITY)).changed();
    changed
}
