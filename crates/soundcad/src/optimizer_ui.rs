//! Optimizer settings panel (target curve, range, penalties, rounding).

use eframe::egui;
use soundcad_core::io::MagUnit;
use soundcad_speaker::optimize::{Edge, Measure, Secondary, SecondaryTarget, Settings, Slope};
use soundcad_speaker::schematic::Snap;

fn edge_row(ui: &mut egui::Ui, label: &str, edge: &mut Option<Edge>, default_fc: f64) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        let mut on = edge.is_some();
        if ui.checkbox(&mut on, label).changed() {
            *edge = on.then_some(Edge { slope: Slope::Butterworth, order: 2, fc: default_fc });
            changed = true;
        }
        if let Some(e) = edge {
            egui::ComboBox::from_id_salt(label).selected_text(if e.slope == Slope::Butterworth { "Butterworth" } else { "Linkwitz-Riley" }).show_ui(ui, |ui| {
                changed |= ui.selectable_value(&mut e.slope, Slope::Butterworth, "Butterworth").changed();
                changed |= ui.selectable_value(&mut e.slope, Slope::LinkwitzRiley, "Linkwitz-Riley").changed();
            });
            changed |= ui.add(egui::DragValue::new(&mut e.order).range(1..=8).prefix("order ")).changed();
            if e.slope == Slope::LinkwitzRiley && e.order % 2 == 1 {
                e.order += 1;
            }
            changed |= ui.add(egui::DragValue::new(&mut e.fc).speed(10.0).range(20.0..=20000.0).suffix(" Hz")).changed();
        }
    });
    changed
}

/// An optional limit: check box, and its value while on (`default` when switched on); true when edited.
fn limit_row(ui: &mut egui::Ui, label: &str, v: &mut Option<f64>, default: f64, suffix: &str, range: std::ops::RangeInclusive<f64>) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        let mut on = v.is_some();
        if ui.checkbox(&mut on, label).changed() {
            *v = on.then_some(default);
            changed = true;
        }
        if let Some(x) = v {
            changed |= ui.add(egui::DragValue::new(x).speed(0.1).range(range).suffix(suffix)).changed();
        }
    });
    changed
}

/// Edits the target and options (a file error goes to `status`); returns true when something changed.
pub fn settings_ui(ui: &mut egui::Ui, s: &mut Settings, status: &mut String) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label("Optimize");
        changed |= ui.selectable_value(&mut s.measure, Measure::RefAxis, "Ref axis response").changed();
        changed |= ui.selectable_value(&mut s.measure, Measure::ListeningWindow, "Listening window").changed();
    });
    egui::Grid::new("optgrid").num_columns(2).show(ui, |ui| {
        ui.label("Target level (1 kHz)");
        changed |= ui.add(egui::DragValue::new(&mut s.level).speed(0.1).suffix(" dB")).changed();
        ui.end_row();
        ui.label("Tilt");
        changed |= ui.add(egui::DragValue::new(&mut s.tilt).speed(0.05).suffix(" dB/oct")).changed();
        ui.end_row();
        ui.label("Frequency range");
        ui.horizontal(|ui| {
            changed |= ui.add(egui::DragValue::new(&mut s.f_lo).speed(10.0).range(20.0..=20000.0).suffix(" Hz")).changed();
            ui.label("…");
            changed |= ui.add(egui::DragValue::new(&mut s.f_hi).speed(10.0).range(20.0..=20000.0).suffix(" Hz")).changed();
        });
        ui.end_row();
    });
    // target from a response file replaces level, tilt and edges
    ui.horizontal(|ui| {
        match &s.target_file {
            Some(r) => {
                ui.label(format!("Target file: {}", r.name));
                changed |= ui.add(egui::DragValue::new(&mut s.file_offset).speed(0.1).suffix(" dB")).changed();
                if ui.button("Clear").clicked() {
                    s.target_file = None;
                    changed = true;
                }
            }
            None => {
                if ui.button("Target from file…").clicked() {
                    match crate::pick_response(MagUnit::Db) {
                        Some(Ok(r)) => {
                            s.target_file = Some(r);
                            changed = true;
                        }
                        Some(Err(e)) => *status = e,
                        None => {}
                    }
                }
            }
        }
    });
    changed |= edge_row(ui, "High-pass", &mut s.hp, 100.0);
    changed |= edge_row(ui, "Low-pass", &mut s.lp, 3000.0);
    changed |= ui.checkbox(&mut s.seek_level, "Seek level (otherwise only the shape)").changed();
    changed |= limit_row(ui, "Minimum impedance", &mut s.min_z, 3.0, " Ω", 0.5..=30.0);
    // second response, optimized together with the main one
    ui.horizontal(|ui| {
        let mut on = s.secondary.is_some();
        if ui.checkbox(&mut on, "Also optimize").changed() {
            s.secondary = on.then_some(SecondaryTarget { kind: Secondary::InRoom, weight: 0.4, level: 0.0, tilt: -0.5 });
            changed = true;
        }
        if let Some(t) = &mut s.secondary {
            changed |= ui.selectable_value(&mut t.kind, Secondary::Power, "Power response").changed();
            changed |= ui.selectable_value(&mut t.kind, Secondary::InRoom, "Predicted in-room").changed();
        }
    });
    if let Some(t) = &mut s.secondary {
        ui.horizontal(|ui| {
            let mut pct = t.weight * 100.0;
            if ui.add(egui::DragValue::new(&mut pct).range(0.0..=100.0).suffix(" %").prefix("weight ")).changed() {
                t.weight = pct / 100.0;
                changed = true;
            }
            changed |= ui.add(egui::DragValue::new(&mut t.level).speed(0.1).suffix(" dB").prefix("level ")).changed();
            changed |= ui.add(egui::DragValue::new(&mut t.tilt).speed(0.05).suffix(" dB/oct").prefix("tilt ")).changed();
        });
        ui.label(egui::RichText::new("Target = main target + level + tilt; weight = share of the error.").weak());
    }
    changed |= limit_row(ui, "Maximum filter gain", &mut s.max_gain, 0.0, " dB", -30.0..=20.0);
    changed |= limit_row(ui, "Reference-axis linearity", &mut s.linearity, 6.0, " dB", 0.5..=40.0);
    ui.horizontal(|ui| {
        ui.label("Round to");
        for (v, l) in [(None, "none"), (Some(Snap::E12), "E12"), (Some(Snap::E24), "E24"), (Some(Snap::E48), "E48")] {
            changed |= ui.selectable_value(&mut s.round, v, l).changed();
        }
        ui.label("Max evaluations");
        changed |= ui.add(egui::DragValue::new(&mut s.max_evals).range(20..=3000)).changed();
    });
    changed
}
