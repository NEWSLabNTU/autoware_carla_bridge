use carla::client::{Actor, ActorKind};

use super::{
    other_bridge::OtherActorBridge,
    sensor_bridge::{SensorBridge, SensorType},
    trafficlight_bridge::TrafficLightBridge,
    trafficsign_bridge::TrafficSignBridge,
    vehicle_bridge::VehicleBridge,
};
use crate::{autoware::Autoware, error::Result};

#[derive(Debug)]
pub enum BridgeType {
    Vehicle(String),
    Sensor(String, SensorType, String),
    TrafficLight,
    TrafficSign,
    Other,
}

pub trait ActorBridge {
    fn step(&mut self, timestamp: f64) -> Result<()>;
}

pub fn get_bridge_type(actor: Actor) -> Result<BridgeType> {
    Ok(match actor.into_kinds() {
        ActorKind::Vehicle(vehicle) => VehicleBridge::get_bridge_type(vehicle)?,
        ActorKind::Sensor(sensor) => SensorBridge::get_bridge_type(sensor)?,
        ActorKind::TrafficLight(_) => BridgeType::TrafficLight,
        ActorKind::TrafficSign(_) => BridgeType::TrafficSign,
        ActorKind::Other(_) => BridgeType::Other,
    })
}

pub fn create_bridge(
    node: rclrs::Node, // Node is already Arc<NodeState>
    actor: Actor,
    bridge_type: BridgeType,
    autoware: &Autoware,
) -> Result<Box<dyn ActorBridge>> {
    Ok(match actor.into_kinds() {
        ActorKind::Vehicle(vehicle) => {
            Box::new(VehicleBridge::new(node, vehicle, bridge_type, autoware)?)
        }
        ActorKind::Sensor(sensor) => {
            Box::new(SensorBridge::new(node, sensor, bridge_type, autoware)?)
        }
        ActorKind::TrafficLight(traffic_light) => {
            Box::new(TrafficLightBridge::new(node, traffic_light)?)
        }
        ActorKind::TrafficSign(traffic_sign) => {
            Box::new(TrafficSignBridge::new(node, traffic_sign)?)
        }
        ActorKind::Other(other) => Box::new(OtherActorBridge::new(node, other)?),
    })
}
