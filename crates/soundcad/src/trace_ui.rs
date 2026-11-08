//! Graph Trace window: a datasheet graph (image file or clipboard), cursors for the two axes, a curve traced by colour
//! or by hand. Image and scales stay for the next curve, since a datasheet shows them all in one graph.

use eframe::egui::{self, Align2, Color32, FontId, Pos2, Rect, Sense, Stroke, Vec2};
use soundcad_core::Response;
use soundcad_tools::spltrace::{follow, impedance_response, join, pick, spl_response, to_points, Axis, Bitmap};

/// What the curve being traced will become.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Target {
    /// SPL at this angle, degrees.
    Spl(f64),
    Impedance,
}

#[derive(Clone, Copy, PartialEq)]
enum Tool {
    Trace,
    Manual,
    Erase,
}

#[derive(Clone, Copy, PartialEq)]
enum Cursor {
    FreqLow,
    FreqHigh,
    ValueLow,
    ValueHigh,
}

const MAGENTA: Color32 = Color32::from_rgb(220, 0, 220);
const ORANGE: Color32 = Color32::from_rgb(240, 130, 0);
const BLUE: Color32 = Color32::from_rgb(40, 90, 255);
const GREEN: Color32 = Color32::from_rgb(0, 170, 60);
const RED: Color32 = Color32::from_rgb(230, 40, 40);
const CYAN: Color32 = Color32::from_rgb(0, 180, 200);

pub struct TraceUi {
    target: Option<Target>,
    bitmap: Option<Bitmap>,
    texture: Option<egui::TextureHandle>,
    name: String,
    x: Axis,
    spl: Axis,
    z: Axis,
    path: Vec<(f64, f64)>,
    tool: Tool,
    tolerance: f32,
    smooth: bool,
    /// Re typed by the user; without it 95 % of the lowest traced impedance
    re: Option<f64>,
    moving: Option<Cursor>,
    zoom: f32,
    pan: Vec2,
    status: String,
}

impl Default for TraceUi {
    fn default() -> Self {
        let axis = |v0, v1, log| Axis { px0: 0.0, px1: 0.0, v0, v1, log };
        TraceUi {
            target: None,
            bitmap: None,
            texture: None,
            name: String::new(),
            x: axis(20.0, 20000.0, true),
            spl: axis(50.0, 110.0, false),
            z: axis(0.0, 80.0, false),
            path: vec![],
            tool: Tool::Trace,
            tolerance: 80.0,
            smooth: true,
            re: None,
            moving: None,
            zoom: 1.0,
            pan: Vec2::ZERO,
            status: String::new(),
        }
    }
}

/// Very large images are reduced: the texture limit of the renderer, and more pixels add nothing.
const MAX_SIDE: u32 = 4096;

fn load_file(path: &std::path::Path) -> Result<Bitmap, String> {
    let img = image::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let img = if img.width() > MAX_SIDE || img.height() > MAX_SIDE { img.thumbnail(MAX_SIDE, MAX_SIDE) } else { img };
    let img = img.to_rgba8();
    Ok(Bitmap { width: img.width() as usize, height: img.height() as usize, rgba: img.into_raw() })
}

fn load_clipboard() -> Result<Bitmap, String> {
    let data = arboard::Clipboard::new().and_then(|mut c| c.get_image()).map_err(|_| "No image in the clipboard".to_string())?;
    Ok(Bitmap { width: data.width, height: data.height, rgba: data.bytes.into_owned() })
}

impl TraceUi {
    /// Opens the window for a new curve; the image and the scales stay.
    pub fn open(&mut self, target: Target) {
        if self.target != Some(target) {
            self.path.clear();
            self.re = None;
        }
        self.tool = Tool::Trace;
        self.status.clear();
        // each curve starts from the whole graph
        (self.zoom, self.pan) = (1.0, Vec2::ZERO);
        self.target = Some(target);
    }

    pub fn close(&mut self) {
        self.target = None;
    }

    fn set_image(&mut self, ctx: &egui::Context, bmp: Bitmap, name: String) {
        // cursors stay for another image of the same size
        if self.bitmap.as_ref().is_none_or(|b| (b.width, b.height) != (bmp.width, bmp.height)) {
            let (w, h) = (bmp.width as f64, bmp.height as f64);
            (self.x.px0, self.x.px1) = (w * 0.1, w * 0.9);
            for a in [&mut self.spl, &mut self.z] {
                (a.px0, a.px1) = (h * 0.9, h * 0.1);
            }
        }
        let image = egui::ColorImage::from_rgba_unmultiplied([bmp.width, bmp.height], &bmp.rgba);
        self.texture = Some(ctx.load_texture("spl_trace", image, egui::TextureOptions::LINEAR));
        self.bitmap = Some(bmp);
        self.name = name;
        self.path.clear();
        (self.zoom, self.pan) = (1.0, Vec2::ZERO);
        self.status.clear();
    }

    fn paste(&mut self, ctx: &egui::Context) {
        match load_clipboard() {
            Ok(b) => self.set_image(ctx, b, "pasted image".into()),
            Err(e) => self.status = e,
        }
    }

    fn value_axis(&mut self) -> &mut Axis {
        if self.target == Some(Target::Impedance) { &mut self.z } else { &mut self.spl }
    }

    /// The traced curve as a response.
    fn result(&self, target: Target) -> Result<Response, String> {
        let base = self.name.rsplit_once('.').map_or(self.name.as_str(), |n| n.0);
        match target {
            Target::Spl(a) => spl_response(&format!("{base}_{a:.0}deg"), &to_points(&self.path, &self.x, &self.spl), self.smooth),
            Target::Impedance => {
                let points = to_points(&self.path, &self.x, &self.z);
                impedance_response(&format!("{base}_ZR"), &points, self.re.unwrap_or_else(|| default_re(&points)), self.smooth)
            }
        }
    }

    /// The window; returns the finished curve when "Use this curve" is pressed.
    pub fn show(&mut self, ctx: &egui::Context) -> Option<(Target, Response)> {
        let target = self.target?;
        let mut out = None;
        let mut open = true;
        let title = match target {
            Target::Spl(a) => format!("Graph Trace: SPL at {a:.0}°"),
            Target::Impedance => "Graph Trace: impedance".to_string(),
        };
        let width = (ctx.screen_rect().width() - 60.0).min(1200.0);
        let height = (ctx.screen_rect().height() - 90.0).min(800.0);
        egui::Window::new(title).id(egui::Id::new("spl_trace")).open(&mut open).collapsible(false).default_size([width, height]).default_pos([30.0, 60.0]).show(ctx, |ui| {
            // the graph takes all the room there is
            ui.set_min_size(Vec2::new(width - 20.0, height - 60.0));
            if self.top_rows(ui, target) {
                self.canvas(ui, target);
                out = self.bottom_row(ui, target);
            }
        });
        if !open || out.is_some() {
            self.target = None;
        }
        out
    }

    /// Image buttons, scales, tools; false while there is no image.
    fn top_rows(&mut self, ui: &mut egui::Ui, target: Target) -> bool {
        ui.horizontal(|ui| {
            if ui.button("Open image…").clicked() {
                if let Some(p) = rfd::FileDialog::new().add_filter("Image", &["png", "jpg", "jpeg", "bmp", "gif"]).pick_file() {
                    match load_file(&p) {
                        Ok(b) => self.set_image(ui.ctx(), b, p.file_name().map_or(String::new(), |n| n.to_string_lossy().into_owned())),
                        Err(e) => self.status = e,
                    }
                }
            }
            if ui.button("Paste image").on_hover_text("From the clipboard, e.g. a screenshot of the datasheet (Win+Shift+S). Also Ctrl+V.").clicked() || ui.input(|i| i.modifiers.command && i.key_pressed(egui::Key::V)) {
                self.paste(ui.ctx());
            }
            ui.label(egui::RichText::new(&self.name).weak());
            if let Some(b) = &self.bitmap {
                ui.label(egui::RichText::new(format!("{} × {} px", b.width, b.height)).weak());
            }
        });
        if self.bitmap.is_none() {
            ui.label("Open the graph of the datasheet as an image, or paste a screenshot of it.");
            return false;
        }
        let (z, (unit, low, high)) = (target == Target::Impedance, value_style(target));
        ui.horizontal_wrapped(|ui| {
            ui.colored_label(MAGENTA, "■");
            ui.add(egui::DragValue::new(&mut self.x.v0).range(0.001..=1e6).suffix(" Hz"));
            ui.colored_label(ORANGE, "■");
            ui.add(egui::DragValue::new(&mut self.x.v1).range(0.001..=1e6).suffix(" Hz"));
            ui.checkbox(&mut self.x.log, "log f");
            ui.separator();
            let a = self.value_axis();
            ui.colored_label(low, "■");
            ui.add(egui::DragValue::new(&mut a.v0).suffix(unit));
            ui.colored_label(high, "■");
            ui.add(egui::DragValue::new(&mut a.v1).suffix(unit));
            if z {
                ui.checkbox(&mut a.log, "log Z");
            }
        });
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.tool, Tool::Trace, "Trace").on_hover_text("Click the curve at its left end. If the trace stops or goes wrong, click again past the problem.");
            ui.selectable_value(&mut self.tool, Tool::Manual, "Manual").on_hover_text("Click each corner of the curve.");
            ui.selectable_value(&mut self.tool, Tool::Erase, "Erase").on_hover_text("Drag over bad points: the gap is interpolated.");
            ui.label("Colour tolerance");
            ui.add(egui::Slider::new(&mut self.tolerance, 20.0..=160.0).show_value(false));
            if ui.add_enabled(!self.path.is_empty(), egui::Button::new("Clear trace")).clicked() {
                self.path.clear();
            }
            ui.label(egui::RichText::new("wheel = zoom · right drag = move").weak());
        });
        true
    }

    /// The image with cursors and trace.
    fn canvas(&mut self, ui: &mut egui::Ui, target: Target) {
        let (Some(bmp), Some(texture)) = (self.bitmap.clone(), self.texture.clone()) else { return };
        let size = Vec2::new(ui.available_width(), (ui.available_height() - 34.0).max(200.0));
        let (rect, resp) = ui.allocate_exact_size(size, Sense::click_and_drag());
        let p = ui.painter_at(rect);
        p.rect_filled(rect, 0.0, Color32::from_gray(30));
        let img = Vec2::new(bmp.width as f32, bmp.height as f32);
        let fit = (rect.width() / img.x).min(rect.height() / img.y);
        // wheel: zoom around the pointer
        if let Some(q) = resp.hover_pos() {
            let wheel = ui.input(|i| i.raw_scroll_delta.y);
            if wheel != 0.0 {
                let old = fit * self.zoom;
                let o = rect.center() - img * old * 0.5 + self.pan;
                let at = (q - o) / old;
                self.zoom = (self.zoom * if wheel > 0.0 { 1.25 } else { 0.8 }).clamp(1.0, 16.0);
                let new = fit * self.zoom;
                // back at the whole image, centred
                self.pan = if self.zoom <= 1.0 { Vec2::ZERO } else { (q - at * new) - (rect.center() - img * new * 0.5) };
            }
        }
        if resp.dragged_by(egui::PointerButton::Secondary) || resp.dragged_by(egui::PointerButton::Middle) {
            self.pan += resp.drag_delta();
        }
        let s = fit * self.zoom;
        let o = rect.center() - img * s * 0.5 + self.pan;
        let screen = |x: f64, y: f64| Pos2::new(o.x + x as f32 * s, o.y + y as f32 * s);
        let pixel = |q: Pos2| (((q.x - o.x) / s) as f64, ((q.y - o.y) / s) as f64);
        p.image(texture.id(), Rect::from_min_size(o, img * s), Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);

        // cursors
        let (unit, low, high) = value_style(target);
        let a = *self.value_axis();
        let x = self.x;
        let vertical = [(Cursor::FreqLow, x.px0, MAGENTA, x.v0), (Cursor::FreqHigh, x.px1, ORANGE, x.v1)];
        let horizontal = [(Cursor::ValueLow, a.px0, low, a.v0), (Cursor::ValueHigh, a.px1, high, a.v1)];
        for (_, px, col, v) in vertical {
            let sx = screen(px, 0.0).x;
            p.line_segment([Pos2::new(sx, rect.top()), Pos2::new(sx, rect.bottom())], Stroke::new(1.5_f32, col));
            tag(&p, Pos2::new(sx + 3.0, rect.top() + 2.0), format!("{v} Hz"), col);
        }
        for (_, py, col, v) in horizontal {
            let sy = screen(0.0, py).y;
            p.line_segment([Pos2::new(rect.left(), sy), Pos2::new(rect.right(), sy)], Stroke::new(1.5_f32, col));
            tag(&p, Pos2::new(rect.left() + 3.0, sy + 2.0), format!("{v}{unit}"), col);
        }

        // the trace, visible on any page and over any curve
        if self.path.len() > 1 {
            let line: Vec<Pos2> = self.path.iter().map(|&(px, py)| screen(px + 0.5, py + 0.5)).collect();
            p.add(egui::Shape::line(line.clone(), Stroke::new(3.0_f32, Color32::BLACK)));
            p.add(egui::Shape::line(line, Stroke::new(1.5_f32, Color32::YELLOW)));
        } else if let Some(&(px, py)) = self.path.first() {
            p.circle_filled(screen(px + 0.5, py + 0.5), 3.0, Color32::YELLOW);
        }

        // left button: a press on a cursor line moves it, except while erasing
        if resp.drag_started_by(egui::PointerButton::Primary) && self.tool != Tool::Erase {
            if let Some(q) = ui.input(|i| i.pointer.press_origin()).or(resp.interact_pointer_pos()) {
                let near_x = vertical.iter().map(|c| (c.0, (screen(c.1, 0.0).x - q.x).abs()));
                let near_y = horizontal.iter().map(|c| (c.0, (screen(0.0, c.1).y - q.y).abs()));
                self.moving = near_x.chain(near_y).filter(|c| c.1 < 7.0).min_by(|a, b| a.1.total_cmp(&b.1)).map(|c| c.0);
            }
        }
        if resp.dragged_by(egui::PointerButton::Primary) {
            if let Some(q) = resp.interact_pointer_pos() {
                let (qx, qy) = pixel(q);
                match self.moving {
                    Some(Cursor::ValueLow) => self.value_axis().px0 = qy,
                    Some(Cursor::ValueHigh) => self.value_axis().px1 = qy,
                    Some(Cursor::FreqLow) => self.x.px0 = qx,
                    Some(Cursor::FreqHigh) => self.x.px1 = qx,
                    None if self.tool == Tool::Erase => self.path.retain(|&(px, py)| screen(px, py).distance(q) > 8.0),
                    None => {}
                }
            }
        }
        if resp.drag_stopped() {
            self.moving = None;
        }
        if resp.clicked() {
            if let Some(q) = resp.interact_pointer_pos() {
                self.click(&bmp, pixel(q));
            }
        }
        // what the pointer is on
        if let Some(q) = resp.hover_pos() {
            let (qx, qy) = pixel(q);
            p.line_segment([Pos2::new(q.x, rect.top()), Pos2::new(q.x, rect.bottom())], Stroke::new(0.5_f32, Color32::from_gray(120)));
            p.line_segment([Pos2::new(rect.left(), q.y), Pos2::new(rect.right(), q.y)], Stroke::new(0.5_f32, Color32::from_gray(120)));
            let a = self.value_axis();
            if a.is_valid() && self.x.is_valid() {
                let (f, v) = (self.x.value(qx), self.value_axis().value(qy));
                let text = if f >= 1000.0 { format!("{:.2} kHz", f / 1000.0) } else { format!("{f:.1} Hz") };
                tag(&p, q + Vec2::new(12.0, 12.0), format!("{text}   {v:.1}{unit}"), Color32::BLACK);
            }
        }
    }

    fn click(&mut self, bmp: &Bitmap, (x, y): (f64, f64)) {
        match self.tool {
            Tool::Trace => match pick(bmp, x, y, 6, self.tolerance) {
                Some((x0, y0, color)) => {
                    let end = self.x.px0.max(self.x.px1).max(0.0) as usize;
                    join(&mut self.path, follow(bmp, x0, y0, color, self.tolerance, end));
                    self.status = match self.path.last() {
                        Some(&(last, _)) if last + 10.0 < end as f64 => "The trace stopped: click the curve again past that point (or use Manual).".into(),
                        _ => String::new(),
                    };
                }
                None => self.status = "No curve here: click on its line.".into(),
            },
            Tool::Manual => {
                let k = self.path.partition_point(|p| p.0 < x);
                self.path.insert(k, (x, y));
            }
            Tool::Erase => self.path.retain(|p| (p.0 - x).hypot(p.1 - y) > 4.0),
        }
    }

    /// Options and the button that finishes the curve.
    fn bottom_row(&mut self, ui: &mut egui::Ui, target: Target) -> Option<(Target, Response)> {
        let mut out = None;
        ui.horizontal(|ui| {
            let scales = self.x.is_valid() && self.value_axis().is_valid();
            if ui.add_enabled(scales && self.path.len() >= 2, egui::Button::new("Use this curve")).clicked() {
                match self.result(target) {
                    Ok(r) => {
                        out = Some((target, r));
                        self.path.clear();
                        self.re = None;
                    }
                    Err(e) => self.status = e,
                }
            }
            ui.checkbox(&mut self.smooth, "Smooth 1/12 oct");
            if target == Target::Impedance && self.path.len() >= 2 && self.z.is_valid() && self.x.is_valid() {
                let points = to_points(&self.path, &self.x, &self.z);
                let mut re = self.re.unwrap_or_else(|| default_re(&points));
                ui.label("Re");
                if ui.add(egui::DragValue::new(&mut re).speed(0.01).range(0.01..=1000.0).suffix(" Ω")).on_hover_text("Voice coil resistance: the impedance below the traced range does not go under it. Default: 95 % of the lowest traced value.").changed() {
                    self.re = Some(re);
                }
            }
            if !scales {
                ui.colored_label(ui.visuals().warn_fg_color, "Check the scales: two different values, above zero on a log axis.");
            }
            ui.label(egui::RichText::new(&self.status).color(ui.visuals().warn_fg_color));
        });
        out
    }
}

/// Unit and the colours of the low and high value cursors.
fn value_style(target: Target) -> (&'static str, Color32, Color32) {
    if target == Target::Impedance { (" Ω", RED, CYAN) } else { (" dB", BLUE, GREEN) }
}

/// 95 % of the lowest traced impedance.
fn default_re(points: &[(f64, f64)]) -> f64 {
    0.95 * points.iter().map(|q| q.1).fold(f64::INFINITY, f64::min).max(0.01)
}

/// Small label on a pale background, readable over any image.
fn tag(p: &egui::Painter, at: Pos2, text: String, col: Color32) {
    let galley = p.layout_no_wrap(text, FontId::proportional(11.0), col);
    let r = Align2::LEFT_TOP.anchor_size(at, galley.size()).expand(1.0);
    p.rect_filled(r, 2.0, Color32::from_rgba_unmultiplied(255, 255, 255, 210));
    p.galley(r.min + Vec2::splat(1.0), galley, col);
}
