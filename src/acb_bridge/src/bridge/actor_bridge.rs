//! What a bridged actor is, and what every bridge must do.
//!
//! This used to carry a dispatcher over five actor kinds, with a bridge implementation per
//! kind. Only the sensor path was ever reached: the ego's control and status live in
//! `vehicle_control.rs`, traffic lights are owned by carla-scenario-bridge, and nothing ever
//! constructed the traffic-sign or "other" bridges. The dispatcher and the unreachable
//! bridges were removed; see docs/roadmap/9-gap-analysis.md gap 2.
#![allow(dead_code)]


use super::sensor_bridge::SensorType;
use crate::error::Result;

/// Which kind of bridge an actor gets. Only `Sensor` is constructed today; the rest are
/// kept because `SensorBridge::get_bridge_type` returns this type and callers match on it.
#[derive(Debug)]
pub enum BridgeType {
    Vehicle,
    Sensor(SensorType, String),
    TrafficSign,
    Other,
}

pub trait ActorBridge {
    fn step(&mut self, timestamp: f64) -> Result<()>;
}


