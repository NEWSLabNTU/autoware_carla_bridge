//! Actor bridge trait and factory functions
//!
//! This module provides the core ActorBridge trait and factory functions for creating
//! bridges for different actor types (Vehicle, Sensor, TrafficLight, TrafficSign, Other).
//!
//! Currently unused but kept for future phases when we implement full actor lifecycle
//! management with Autoware integration.
#![allow(dead_code)]

use carla::client::ActorKind;

use super::{
    other_bridge::OtherActorBridge,
    sensor_bridge::{SensorBridge, SensorType},
    trafficsign_bridge::TrafficSignBridge,
    vehicle_bridge::VehicleBridge,
};
use crate::{
    autoware::Autoware,
    error::{BridgeError, Result},
};

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

pub fn get_bridge_type(actor_kind: ActorKind) -> Result<BridgeType> {
    Ok(match actor_kind {
        ActorKind::Vehicle(vehicle) => VehicleBridge::get_bridge_type(vehicle)?,
        ActorKind::Sensor(sensor) => SensorBridge::get_bridge_type(sensor)?,
        // Traffic lights are deliberately not bridged: carla-scenario-bridge is the sole
        // traffic-light writer, and a bridge here would be a second authority on signal
        // state.
        ActorKind::TrafficLight(_) => {
            return Err(BridgeError::CarlaIssue(
                "traffic lights are not bridged; carla-scenario-bridge owns signal state",
            ))
        }
        ActorKind::TrafficSign(_) => BridgeType::TrafficSign,
        ActorKind::Other(_) => BridgeType::Other,
    })
}

pub fn create_bridge(
    node: rclrs::Node, // Node is already Arc<NodeState>
    actor_kind: ActorKind,
    bridge_type: BridgeType,
    autoware: &Autoware,
    clock_offset: crate::utils::SimClockOffset,
) -> Result<Box<dyn ActorBridge>> {
    Ok(match actor_kind {
        ActorKind::Vehicle(vehicle) => {
            Box::new(VehicleBridge::new(node, vehicle, bridge_type, autoware)?)
        }
        ActorKind::Sensor(sensor) => {
            Box::new(SensorBridge::new(node, sensor, bridge_type, autoware, clock_offset)?)
        }
        ActorKind::TrafficLight(_) => {
            return Err(BridgeError::CarlaIssue(
                "traffic lights are not bridged; carla-scenario-bridge owns signal state",
            ))
        }
        ActorKind::TrafficSign(traffic_sign) => {
            Box::new(TrafficSignBridge::new(node, traffic_sign)?)
        }
        ActorKind::Other(other) => Box::new(OtherActorBridge::new(node, other)?),
    })
}
