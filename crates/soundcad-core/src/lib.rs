//! Core data model shared by every SoundCAD tool and node.
//!
//! A [`Response`] is a complex transfer function sampled on a frequency grid
//! (sound pressure, impedance, voltage, a diffraction ratio, ...). All tools
//! consume and produce responses, which is what makes them composable as nodes.

pub mod angle;
pub mod io;
pub mod minphase;
pub mod response;
pub mod smooth;

pub use num_complex::Complex64;
pub use response::{Plane, Response};

/// Air density [kg/m³] and speed of sound [m/s] of every model.
pub const RHO: f64 = 1.194;
pub const C_AIR: f64 = 344.0;
