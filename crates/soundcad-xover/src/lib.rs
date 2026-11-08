//! Crossover simulator: project model, schematic netlist and circuit solver.
//!
//! Implemented so far: passive networks (generator, R, C, L, wires, grounds, drivers as measured
//! impedance).

pub mod circuit;
pub mod project;
pub mod sim;

pub use circuit::{solve, Solution};
pub use project::{Part, PartKind, Project};
pub use sim::{group_delay, simulate_with, Cta, DriverResponses, PowerOptions, Simulation};
