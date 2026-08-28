//! Platform-independent flight estimation and control.
//!
//! Hardware access stays in the firmware crate. This crate receives
//! measurements as ordinary values and returns estimates and actuator
//! requests as ordinary values.

pub mod altitude_controller;
pub mod attitude_controller;
pub mod attitude_estimator;
pub mod collective;
pub mod flight_controller;
pub mod flight_state;
pub mod flow_measurement;
pub mod horizontal_controller;
pub mod horizontal_estimator;
pub mod math;
pub mod mixer;
pub mod pid;
pub mod safety;
pub mod types;
pub mod vertical_acceleration;
pub mod vertical_estimator;
