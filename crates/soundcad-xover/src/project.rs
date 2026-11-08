//! Crossover project model: drivers with their measured responses, schematic parts, room and listening setup.

use std::collections::HashMap;


#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum PartKind {
    Generator,
    Ground,
    Driver,
    Resistor,
    Capacitor,
    Inductor,
    Wire,
    /// Active blocks, transformers, library blocks, ... not simulated yet.
    Other,
}

/// One schematic component. Terminals are the first two `WIRE` points (a `Wire` part connects all its points).
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Part {
    pub kind: PartKind,
    pub type_name: String,
    pub id: String,
    /// Driver model name for `Driver` parts.
    pub model: String,
    pub params: HashMap<String, f64>,
    pub points: Vec<(i32, i32)>,
    pub open: bool,
    pub shorted: bool,
    pub inverted: bool,
    pub muted: bool,
}

impl Part {
    pub fn param(&self, name: &str) -> Option<f64> {
        self.params.get(name).copied()
    }
}

/// Driver of the crossover project: nominal Z, response files and per-driver settings.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct DriverData {
    pub model: String,
    /// Nominal Z [Ω], used when no impedance is loaded.
    pub z: f64,
    pub impedance_scale: f64,
    pub scale: f64,
    pub delay_us: f64,
    pub invert: bool,
    pub mute: bool,
}

/// Room: 1st-order reflecting surfaces (mm from the speaker origin; left wall X and floor Y
/// negative), absorption in dB, toe-in in degrees.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct Room {
    pub front_wall: bool,
    pub front_wall_z: f64,
    pub left_wall: bool,
    pub left_wall_x: f64,
    pub ceiling: bool,
    pub ceiling_y: f64,
    pub floor: bool,
    pub floor_y: f64,
    pub toe_in: f64,
    pub absorp_wall: f64,
    pub absorp_ceil: f64,
    pub absorp_floor: f64,
    /// Simulated planes rotated around Z [deg, ccw], temporary X/Y offset of all
    /// drivers [mm].
    pub plane_rotation: f64,
    pub offset_x: f64,
    pub offset_y: f64,
}

#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct Project {
    pub drivers: Vec<DriverData>,
    pub parts: Vec<Part>,
    pub room: Room,
}

