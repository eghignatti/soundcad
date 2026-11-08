//! Graphic crossover editor: toolbar to place parts, drag to move, R to rotate,
//! Delete to remove, mouse wheel over a part to step its value through the chosen series (5 %, E12,
//! E24, E48), values printed on the parts and editable in the panel below. Drag empty space to pan,
//! wheel over empty space to zoom.

use eframe::egui::{self, Align2, Color32, FontId, Key, Pos2, Rect, Sense, Stroke, Vec2};
use soundcad_speaker::schematic::{step, step_from_zero, Comp, Kind, Schematic, Snap, DRIVER_LEN, PART_LEN};
use soundcad_speaker::Unit;

#[derive(Clone, Copy, PartialEq)]
enum Tool {
    Place(Kind),
    Wire,
}

pub struct Editor {
    tool: Option<Tool>,
    selected: Option<usize>,
    wire_start: Option<(i32, i32)>,
    /// part being moved and the grid point where the drag started
    moving: Option<(usize, (i32, i32))>,
    panning: bool,
    /// pixels per grid unit, pan of the grid origin
    scale: f32,
    pan: Vec2,
    fit: bool,
    pub snap: Snap,
    /// unit chosen for the next driver placed
    driver_pick: usize,
}

impl Default for Editor {
    fn default() -> Self {
        Editor { tool: None, selected: None, wire_start: None, moving: None, panning: false, scale: 22.0, pan: Vec2::ZERO, fit: true, snap: Snap::E12, driver_pick: 0 }
    }
}

const ACCENT: Color32 = Color32::from_rgb(90, 170, 250);

/// Value with its unit prefix: 10Ω, 280mΩ, 8.2µF, 680µH, 3.9mH.
pub fn fmt_value(kind: Kind, v: f64) -> String {
    let num = |x: f64| {
        let s = if x >= 100.0 { format!("{x:.0}") } else if x >= 10.0 { format!("{x:.1}") } else { format!("{x:.2}") };
        if s.contains('.') { s.trim_end_matches('0').trim_end_matches('.').to_string() } else { s }
    };
    match kind {
        Kind::Resistor => if v < 1.0 { format!("{}mΩ", num(v * 1000.0)) } else { format!("{}Ω", num(v)) },
        Kind::Capacitor => format!("{}µF", num(v)),
        Kind::Inductor => if v < 1.0 { format!("{}µH", num(v * 1000.0)) } else { format!("{}mH", num(v)) },
        _ => String::new(),
    }
}

fn loss_text(v: f64) -> String {
    if v <= 0.0 { String::new() } else if v < 1.0 { format!("{:.0}mΩ", v * 1000.0) } else { format!("{v:.2}Ω") }
}

/// Distance from `p` to segment `a`–`b`.
fn seg_dist(p: Pos2, a: Pos2, b: Pos2) -> f32 {
    let ab = b - a;
    let t = if ab.length_sq() == 0.0 { 0.0 } else { ((p - a).dot(ab) / ab.length_sq()).clamp(0.0, 1.0) };
    (a + ab * t).distance(p)
}

impl Editor {
    /// Draws and edits the schematic; returns true when the circuit changed.
    pub fn ui(&mut self, ui: &mut egui::Ui, sch: &mut Schematic, unit_list: &mut [Unit]) -> bool {
        let mut changed = false;
        let units: Vec<String> = unit_list.iter().map(|u| u.name.clone()).collect();
        // toolbar
        ui.horizontal_wrapped(|ui| {
            for (label, tool) in [
                ("R", Tool::Place(Kind::Resistor)),
                ("C", Tool::Place(Kind::Capacitor)),
                ("L", Tool::Place(Kind::Inductor)),
                ("Wire", Tool::Wire),
                ("Ground", Tool::Place(Kind::Ground)),
                ("Driver", Tool::Place(Kind::Driver)),
            ] {
                if ui.selectable_label(self.tool == Some(tool), label).clicked() {
                    self.tool = if self.tool == Some(tool) { None } else { Some(tool) };
                    self.wire_start = None;
                }
            }
            if self.tool == Some(Tool::Place(Kind::Driver)) && !units.is_empty() {
                self.driver_pick = self.driver_pick.min(units.len() - 1);
                egui::ComboBox::from_id_salt("drv_pick").selected_text(units[self.driver_pick].clone()).show_ui(ui, |ui| {
                    for (i, n) in units.iter().enumerate() {
                        ui.selectable_value(&mut self.driver_pick, i, n);
                    }
                });
            }
            ui.separator();
            if ui.small_button("Fit").clicked() {
                self.fit = true;
            }
            ui.label("Snap");
            for (s, l) in [(Snap::Percent5, "5%"), (Snap::E12, "E12"), (Snap::E24, "E24"), (Snap::E48, "E48")] {
                ui.selectable_value(&mut self.snap, s, l);
            }
        });

        let h = (ui.available_height() * 0.62).max(200.0);
        let (rect, resp) = ui.allocate_exact_size(Vec2::new(ui.available_width(), h), Sense::click_and_drag());
        let p = ui.painter_at(rect);
        p.rect_filled(rect, 4.0, Color32::from_gray(245));

        if self.fit {
            self.fit = false;
            let pts: Vec<(i32, i32)> = sch.parts.iter().flat_map(|c| [c.a, c.b]).collect();
            if let (Some(x0), Some(x1), Some(y0), Some(y1)) =
                (pts.iter().map(|q| q.0).min(), pts.iter().map(|q| q.0).max(), pts.iter().map(|q| q.1).min(), pts.iter().map(|q| q.1).max())
            {
                let (w, hh) = ((x1 - x0 + 4) as f32, (y1 - y0 + 4) as f32);
                self.scale = (rect.width() / w).min(rect.height() / hh).clamp(8.0, 40.0);
                self.pan = Vec2::new(-(x0 as f32 - 2.0) * self.scale, -(y0 as f32 - 2.0) * self.scale);
            }
        }
        let s = self.scale;
        let origin = rect.min + self.pan;
        let to_screen = |g: (i32, i32)| origin + Vec2::new(g.0 as f32 * s, g.1 as f32 * s);
        let to_grid = |q: Pos2| (((q.x - origin.x) / s).round() as i32, ((q.y - origin.y) / s).round() as i32);

        // grid dots at every zoom: small squares in one mesh, thousands of circles are too slow without a GPU
        let (g0, g1) = (to_grid(rect.min), to_grid(rect.max));
        let mut dots = egui::Mesh::default();
        for gx in g0.0..=g1.0 {
            for gy in g0.1..=g1.1 {
                dots.add_colored_rect(Rect::from_center_size(to_screen((gx, gy)), Vec2::splat(2.0)), Color32::from_gray(135));
            }
        }
        p.add(dots);

        // hovered part (for selection, drag, wheel)
        let hover = resp.hover_pos();
        let hit = |parts: &[Comp], q: Pos2| -> Option<usize> {
            parts
                .iter()
                .enumerate()
                .filter(|(_, c)| {
                    let (a, b) = (to_screen(c.a), to_screen(c.b));
                    match c.kind {
                        Kind::Ground => q.distance(a + Vec2::new(0.0, 0.4 * s)) < 0.6 * s,
                        Kind::Wire => seg_dist(q, a, b) < (0.25 * s).max(5.0),
                        _ => seg_dist(q, a, b) < (0.6 * s).max(8.0),
                    }
                })
                // wires last: a part wins over the wire under it
                .min_by_key(|(_, c)| c.kind == Kind::Wire)
                .map(|(i, _)| i)
        };

        // draw parts
        let mut ends: std::collections::HashMap<(i32, i32), usize> = Default::default();
        for c in &sch.parts {
            *ends.entry(c.a).or_default() += 1;
            if c.kind != Kind::Ground {
                *ends.entry(c.b).or_default() += 1;
            }
        }
        for (i, c) in sch.parts.iter().enumerate() {
            let col = if self.selected == Some(i) { ACCENT } else { Color32::from_gray(40) };
            if c.opt {
                // adjusted by the optimizer: pale green band under the part
                p.line_segment([to_screen(c.a), to_screen(c.b)], Stroke::new(0.9 * s, Color32::from_rgba_unmultiplied(110, 215, 130, 110)));
            }
            draw_part(&p, c, to_screen(c.a), to_screen(c.b), s, col);
        }
        for (g, n) in &ends {
            if *n >= 3 {
                p.circle_filled(to_screen(*g), 0.18 * s, Color32::from_gray(40));
            }
        }

        // placing preview
        if let (Some(Tool::Place(kind)), Some(q)) = (self.tool, hover) {
            let g = to_grid(q);
            let ghost = new_part(kind, g, units.get(self.driver_pick).cloned().unwrap_or_default());
            draw_part(&p, &ghost, to_screen(ghost.a), to_screen(ghost.b), s, Color32::from_rgba_unmultiplied(90, 170, 250, 140));
        }
        if let (Some(Tool::Wire), Some(start), Some(q)) = (self.tool, self.wire_start, hover) {
            let g = to_grid(q);
            let corner = (g.0, start.1);
            let ghost = Stroke::new(2.0_f32, ACCENT);
            p.line_segment([to_screen(start), to_screen(corner)], ghost);
            p.line_segment([to_screen(corner), to_screen(g)], ghost);
        }

        // clicks
        if resp.clicked() {
            if let Some(q) = resp.interact_pointer_pos() {
                let g = to_grid(q);
                match self.tool {
                    Some(Tool::Place(kind)) => {
                        sch.parts.push(new_part(kind, g, units.get(self.driver_pick).cloned().unwrap_or_default()));
                        self.selected = Some(sch.parts.len() - 1);
                        self.tool = None;
                        changed = true;
                    }
                    Some(Tool::Wire) => match self.wire_start {
                        None => self.wire_start = Some(g),
                        Some(a) if a != g => {
                            // L-shaped: horizontal first, then vertical
                            let corner = (g.0, a.1);
                            for (u, v) in [(a, corner), (corner, g)] {
                                if u != v {
                                    sch.parts.push(Comp::new(Kind::Wire, u, v, 0.0));
                                }
                            }
                            self.wire_start = None;
                            changed = true;
                        }
                        _ => {}
                    },
                    None => self.selected = hit(&sch.parts, q),
                }
            }
        }
        // drag: move a part, or pan
        if resp.drag_started() && self.tool.is_none() {
            // egui reports the drag after a few pixels of movement: hit-test where the button went down
            if let Some(q) = ui.input(|i| i.pointer.press_origin()).or(resp.interact_pointer_pos()) {
                match hit(&sch.parts, q) {
                    Some(i) => {
                        self.selected = Some(i);
                        self.moving = Some((i, to_grid(q)));
                    }
                    None => self.panning = true,
                }
            }
        }
        if resp.dragged() {
            if self.panning {
                self.pan += resp.drag_delta();
            } else if let (Some((i, from)), Some(q)) = (self.moving, resp.interact_pointer_pos()) {
                let g = to_grid(q);
                let d = (g.0 - from.0, g.1 - from.1);
                if d != (0, 0) {
                    let c = &mut sch.parts[i];
                    c.a = (c.a.0 + d.0, c.a.1 + d.1);
                    c.b = (c.b.0 + d.0, c.b.1 + d.1);
                    self.moving = Some((i, g));
                    changed = true;
                }
            }
        }
        if resp.drag_stopped() {
            self.moving = None;
            self.panning = false;
        }

        // wheel: value of the part under the cursor, else zoom
        if let Some(q) = hover {
            let wheel = ui.input(|i| i.raw_scroll_delta.y);
            if wheel != 0.0 {
                match hit(&sch.parts, q).filter(|&i| sch.parts[i].has_value()) {
                    Some(i) => {
                        let c = &mut sch.parts[i];
                        c.value = step(c.value, wheel > 0.0, self.snap);
                        self.selected = Some(i);
                        changed = true;
                    }
                    None => {
                        let k = if wheel > 0.0 { 1.15 } else { 1.0 / 1.15 };
                        let ns = (self.scale * k).clamp(6.0, 60.0);
                        // zoom around the cursor
                        self.pan = (q - rect.min) - (q - origin) * (ns / self.scale);
                        self.scale = ns;
                    }
                }
            }
        }

        // keys while the pointer is over the drawing (not while typing in the table: Del there is for the text)
        if resp.hovered() && !ui.ctx().wants_keyboard_input() {
            let (del, rot, esc) = ui.input(|i| (i.key_pressed(Key::Delete), i.key_pressed(Key::R), i.key_pressed(Key::Escape)));
            if esc {
                self.tool = None;
                self.wire_start = None;
            }
            if let Some(i) = self.selected.filter(|&i| i < sch.parts.len()) {
                if del {
                    sch.parts.remove(i);
                    self.selected = None;
                    changed = true;
                } else if rot && sch.parts[i].kind != Kind::Ground {
                    sch.parts[i].rotate();
                    changed = true;
                }
            }
        }
        let hint = match (self.tool, self.wire_start) {
            (Some(Tool::Wire), None) => "click the start point of the wire",
            (Some(Tool::Wire), Some(_)) => "click the end point (Esc cancels)",
            (Some(Tool::Place(_)), _) => "click where to place it (Esc cancels)",
            _ => "drag to move · R rotate · Del delete · wheel on a part = next value · wheel elsewhere = zoom",
        };
        p.text(rect.left_bottom() + Vec2::new(6.0, -6.0), Align2::LEFT_BOTTOM, hint, FontId::proportional(11.0), Color32::from_gray(110));

        changed | self.properties(ui, sch, &units, unit_list)
    }

    /// Table of the selected part, below the drawing.
    fn properties(&mut self, ui: &mut egui::Ui, sch: &mut Schematic, names: &[String], units: &mut [Unit]) -> bool {
        let mut changed = false;
        let Some(i) = self.selected.filter(|&i| i < sch.parts.len()) else {
            ui.label(egui::RichText::new("Select a part: its parameters appear here.").weak());
            return false;
        };
        let snap = self.snap;
        let mut delete = false;
        let c = &mut sch.parts[i];
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new(format!("{:?}", c.kind)).strong());
            if c.kind != Kind::Ground && ui.small_button("Rotate").clicked() {
                c.rotate();
                changed = true;
            }
            delete = ui.small_button("Delete").clicked();
        });
        ui.horizontal_top(|ui| {
            egui::Grid::new("part_table").num_columns(2).striped(true).show(ui, |ui| {
                let (series, value) = (|v: f64, up: bool| step(v, up, snap), 0.001..=100000.0);
                match c.kind {
                    Kind::Resistor => changed |= table_row(ui, "Resistance", &mut c.value, " Ω", 0.01, value, series),
                    Kind::Capacitor | Kind::Inductor => {
                        let (name, unit, loss) = if c.kind == Kind::Capacitor { ("Capacitance", " µF", "ESR") } else { ("Inductance", " mH", "DCR") };
                        changed |= table_row(ui, name, &mut c.value, unit, 0.01, value, series);
                        changed |= table_row(ui, loss, &mut c.loss, " Ω", 0.01, 0.0..=10.0, |v, up| step_from_zero(v, up, 0.1, snap));
                    }
                    Kind::Driver => {
                        ui.label("Driver");
                        egui::ComboBox::from_id_salt("drv_of").selected_text(c.driver.clone()).show_ui(ui, |ui| {
                            for n in names {
                                changed |= ui.selectable_value(&mut c.driver, n.clone(), n).changed();
                            }
                        });
                        ui.end_row();
                        ui.label("Polarity");
                        changed |= ui.checkbox(&mut c.inverted, "inverted").changed();
                        ui.end_row();
                        if let Some(u) = units.iter_mut().find(|u| u.name == c.driver) {
                            changed |= table_row(ui, "Depth from the baffle", &mut u.z, " mm", 1.0, -500.0..=500.0, |v, up| v + if up { 1.0 } else { -1.0 });
                        }
                    }
                    _ => {}
                }
                if c.has_value() {
                    ui.label("Optimizer");
                    changed |= ui.checkbox(&mut c.opt, "adjust this part").changed();
                    ui.end_row();
                    if c.opt {
                        // limits for the optimizer: 0 = automatic (value/10 … value×10)
                        let v0 = c.value;
                        changed |= table_row(ui, "Minimum (0 = auto)", &mut c.opt_min, "", 0.01, 0.0..=100000.0, |v, up| step_from_zero(v, up, v0, snap));
                        changed |= table_row(ui, "Maximum (0 = auto)", &mut c.opt_max, "", 0.01, 0.0..=100000.0, |v, up| step_from_zero(v, up, v0, snap));
                    }
                }
            });
            if c.kind == Kind::Driver {
                if let Some(u) = units.iter().find(|u| u.name == c.driver) {
                    ui.add_space(12.0);
                    depth_sketch(ui, u.z);
                }
            }
        });
        if c.has_value() || c.kind == Kind::Driver {
            let text = if c.kind == Kind::Driver { "Wheel over the depth: 1 mm. Position X and Y: Baffle tab." } else { "Wheel over a value: next one of the Snap series." };
            ui.label(egui::RichText::new(text).weak());
        }
        if delete {
            sch.parts.remove(i);
            self.selected = None;
            changed = true;
        }
        changed
    }
}

/// One row of the part table: value typed, dragged or stepped by the wheel with `next(value, up)`.
fn table_row(ui: &mut egui::Ui, label: &str, v: &mut f64, suffix: &str, speed: f64, range: std::ops::RangeInclusive<f64>, next: impl Fn(f64, bool) -> f64) -> bool {
    ui.label(label);
    let r = ui.add(egui::DragValue::new(v).speed(speed).range(range.clone()).max_decimals(3).suffix(suffix));
    let mut changed = r.changed();
    if r.hovered() {
        let wheel = ui.input(|i| i.raw_scroll_delta.y);
        if wheel != 0.0 {
            *v = next(*v, wheel > 0.0).clamp(*range.start(), *range.end());
            changed = true;
        }
    }
    ui.end_row();
    changed
}

/// Example cabinet seen from the side: where a depth of `z` mm puts the acoustic centre (+ = behind the baffle).
fn depth_sketch(ui: &mut egui::Ui, z: f64) {
    let (rect, _) = ui.allocate_exact_size(Vec2::new(250.0, 118.0), Sense::hover());
    let p = ui.painter_at(rect);
    let (line, weak) = (ui.visuals().text_color(), ui.visuals().weak_text_color());
    let font = FontId::proportional(11.0);
    let (bx, top, bottom) = (rect.left() + 95.0, rect.top() + 18.0, rect.bottom() - 34.0);
    let mid = (top + bottom) / 2.0;
    // cabinet, baffle at its left side, driver set into it
    let cabinet = Rect::from_min_max(Pos2::new(bx, top), Pos2::new(rect.right() - 10.0, bottom));
    p.rect_filled(cabinet, 2.0, Color32::from_gray(52));
    p.rect_stroke(cabinet, 2.0, Stroke::new(1.0_f32, weak), egui::StrokeKind::Middle);
    p.add(egui::Shape::closed_line(
        vec![Pos2::new(bx, mid - 20.0), Pos2::new(bx + 18.0, mid - 6.0), Pos2::new(bx + 28.0, mid - 6.0), Pos2::new(bx + 28.0, mid + 6.0), Pos2::new(bx + 18.0, mid + 6.0), Pos2::new(bx, mid + 20.0)],
        Stroke::new(1.0_f32, line),
    ));
    p.line_segment([Pos2::new(bx, top - 4.0), Pos2::new(bx, bottom + 12.0)], Stroke::new(2.0_f32, line));
    p.text(Pos2::new(bx, top - 5.0), Align2::CENTER_BOTTOM, "baffle = 0", font.clone(), line);
    // listener side
    p.text(Pos2::new(rect.left() + 2.0, mid), Align2::LEFT_CENTER, "listener", font.clone(), weak);
    let (a0, a1) = (Pos2::new(rect.left() + 48.0, mid), Pos2::new(bx - 12.0, mid));
    p.line_segment([a0, a1], Stroke::new(1.0_f32, weak));
    p.line_segment([a1, a1 + Vec2::new(-5.0, -3.0)], Stroke::new(1.0_f32, weak));
    p.line_segment([a1, a1 + Vec2::new(-5.0, 3.0)], Stroke::new(1.0_f32, weak));
    // the scale: − in front of the baffle, + behind it
    let scale_y = bottom + 12.0;
    p.line_segment([Pos2::new(bx - 80.0, scale_y), Pos2::new(rect.right() - 10.0, scale_y)], Stroke::new(1.0_f32, weak));
    p.text(Pos2::new(bx - 5.0, scale_y + 3.0), Align2::RIGHT_TOP, "− in front", font.clone(), line);
    p.text(Pos2::new(bx + 5.0, scale_y + 3.0), Align2::LEFT_TOP, "+ behind", font.clone(), line);
    // the acoustic centre, the drawing is not to scale beyond ±60 mm
    let x = bx + z.clamp(-60.0, 60.0) as f32 * 1.1;
    p.line_segment([Pos2::new(x, mid), Pos2::new(x, scale_y)], Stroke::new(1.0_f32, ACCENT));
    p.circle_filled(Pos2::new(x, mid), 3.5, ACCENT);
    p.circle_filled(Pos2::new(x, scale_y), 2.5, ACCENT);
    let (at, align) = if z < 0.0 { (Pos2::new(x - 6.0, mid - 6.0), Align2::RIGHT_BOTTOM) } else { (Pos2::new(x + 6.0, mid - 6.0), Align2::LEFT_BOTTOM) };
    p.text(at, align, format!("{z:.0} mm"), font, ACCENT);
}

/// New part at grid point `g`, horizontal (drivers vertical, + on top).
fn new_part(kind: Kind, g: (i32, i32), driver: String) -> Comp {
    match kind {
        Kind::Resistor => Comp::new(kind, g, (g.0 + PART_LEN, g.1), 10.0),
        Kind::Capacitor => Comp::new(kind, g, (g.0 + PART_LEN, g.1), 10.0),
        Kind::Inductor => Comp { loss: 0.2, ..Comp::new(kind, g, (g.0 + PART_LEN, g.1), 0.47) },
        Kind::Driver => Comp { driver, ..Comp::new(kind, g, (g.0, g.1 + DRIVER_LEN), 0.0) },
        Kind::Generator => Comp::new(kind, g, (g.0, g.1 + DRIVER_LEN), 2.83),
        _ => Comp::new(kind, g, g, 0.0),
    }
}

/// Symbol between terminals `a` and `b` (screen), grid size `s`, with its value printed beside it.
fn draw_part(p: &egui::Painter, c: &Comp, a: Pos2, b: Pos2, s: f32, col: Color32) {
    let st = Stroke::new(1.6_f32, col);
    let len = a.distance(b);
    let u = if len > 0.0 { (b - a) / len } else { Vec2::X };
    let n = Vec2::new(-u.y, u.x);
    let mid = a + (b - a) * 0.5;
    let text = |pos: Pos2, s2: String, al: Align2| {
        p.text(pos, al, s2, FontId::proportional((0.5 * s).clamp(9.0, 15.0)), col);
    };
    // label beside the part: above a horizontal part, left of a vertical one
    let horizontal = u.x.abs() > 0.5;
    let label_at = if horizontal { mid - Vec2::new(0.0, 0.55 * s) } else { mid - Vec2::new(0.55 * s, 0.0) };
    let label_al = if horizontal { Align2::CENTER_BOTTOM } else { Align2::RIGHT_CENTER };
    // leads up to a body of length `body` centred between the terminals
    let leads = |body: f32| {
        let h = (len - body).max(0.0) / 2.0;
        p.line_segment([a, a + u * h], st);
        p.line_segment([b - u * h, b], st);
        (a + u * h, b - u * h)
    };
    match c.kind {
        Kind::Wire => {
            p.line_segment([a, b], Stroke::new(1.6_f32, col));
        }
        Kind::Ground => {
            let base = a + Vec2::new(0.0, 0.35 * s);
            p.line_segment([a, base], st);
            for (k, w) in [0.5_f32, 0.32, 0.14].iter().enumerate() {
                let y = base + Vec2::new(0.0, k as f32 * 0.14 * s);
                p.line_segment([y - Vec2::new(w * s, 0.0), y + Vec2::new(w * s, 0.0)], st);
            }
        }
        Kind::Resistor => {
            let (s0, s1) = leads(1.6 * s);
            let w = n * 0.28 * s;
            p.add(egui::Shape::closed_line(vec![s0 + w, s1 + w, s1 - w, s0 - w], st));
            text(label_at, fmt_value(c.kind, c.value), label_al);
        }
        Kind::Capacitor => {
            let gap = 0.14 * s;
            let (s0, s1) = leads(2.0 * gap);
            for q in [s0, s1] {
                p.line_segment([q - n * 0.45 * s, q + n * 0.45 * s], Stroke::new(2.2_f32, col));
            }
            let mut t = fmt_value(c.kind, c.value);
            if c.loss > 0.0 {
                t += &format!(" {}", loss_text(c.loss));
            }
            text(label_at, t, label_al);
        }
        Kind::Inductor => {
            let body = 1.8 * s;
            let (s0, _) = leads(body);
            // four bumps
            let bumps = 4;
            let r = body / (2.0 * bumps as f32);
            let mut pts = vec![];
            for k in 0..bumps {
                let cx = s0 + u * (r * (2 * k + 1) as f32);
                for j in 0..=8 {
                    let t = std::f32::consts::PI * (1.0 - j as f32 / 8.0);
                    pts.push(cx + u * (r * t.cos()) - n * (r * t.sin()) * if horizontal { 1.0 } else { -1.0 });
                }
            }
            p.add(egui::Shape::line(pts, st));
            text(label_at, fmt_value(c.kind, c.value), label_al);
            if c.loss > 0.0 {
                let below = if horizontal { mid + Vec2::new(0.0, 0.45 * s) } else { label_at + Vec2::new(0.0, 0.55 * s) };
                text(below, loss_text(c.loss), if horizontal { Align2::CENTER_TOP } else { Align2::RIGHT_CENTER });
            }
        }
        Kind::Driver | Kind::Generator => {
            let (s0, s1) = leads(1.6 * s);
            let cen = s0 + (s1 - s0) * 0.5;
            if c.kind == Kind::Generator {
                p.circle_stroke(cen, 0.75 * s, st);
                let pts: Vec<Pos2> = (0..=16).map(|k| cen + Vec2::new((k as f32 / 16.0 - 0.5) * 0.9 * s, -(k as f32 / 16.0 * std::f32::consts::TAU).sin() * 0.25 * s)).collect();
                p.add(egui::Shape::line(pts, st));
                text(cen + Vec2::new(0.95 * s, 0.0), "2.83V".into(), Align2::LEFT_CENTER);
            } else {
                // box + cone
                let box_r = Rect::from_center_size(cen - n * 0.2 * s, Vec2::splat(0.0)).expand2(Vec2::new(0.22 * s, 0.4 * s));
                let r = if horizontal { Rect::from_center_size(box_r.center(), Vec2::new(box_r.height(), box_r.width())) } else { box_r };
                p.rect_stroke(r, 0.0, st, egui::StrokeKind::Middle);
                let side = if horizontal { Vec2::new(0.0, -1.0) } else { Vec2::new(1.0, 0.0) };
                let along = if horizontal { Vec2::new(1.0, 0.0) } else { Vec2::new(0.0, 1.0) };
                let e0 = r.center() + side * (0.22 * s);
                p.add(egui::Shape::closed_line(
                    vec![e0 - along * 0.4 * s, e0 + along * 0.4 * s, e0 + side * 0.5 * s + along * 0.75 * s, e0 + side * 0.5 * s - along * 0.75 * s],
                    st,
                ));
                let name = if c.inverted { format!("{} (inv)", c.driver) } else { c.driver.clone() };
                text(cen + Vec2::new(1.0 * s, 0.0), name, Align2::LEFT_CENTER);
            }
            text(a + u * 0.35 * s + n * 0.45 * s, "+".into(), Align2::CENTER_CENTER);
        }
    }
}
