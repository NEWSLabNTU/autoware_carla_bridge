//! Traffic light bridge - bidirectional state synchronization
//!
//! This module will handle bidirectional traffic light state synchronization
//! between CARLA and Autoware.
//!
//! Currently unused but kept for future phases.
#![allow(dead_code)]

use carla::client::TrafficLight;

use super::actor_bridge::ActorBridge;
use crate::error::Result;

pub struct TrafficLightBridge {
    _actor: TrafficLight,
}

impl TrafficLightBridge {
    pub fn new(_node: rclrs::Node, _actor: TrafficLight) -> Result<TrafficLightBridge> {
        Ok(TrafficLightBridge { _actor })
    }
}

impl ActorBridge for TrafficLightBridge {
    fn step(&mut self, _timestamp: f64) -> Result<()> {
        Ok(())
    }
}
