use std::{
    sync::Arc,
    time::{SystemTime, UNIX_EPOCH},
};

use crate::error::Result;

pub struct SimulatorClock {
    publisher_clock: Arc<rclrs::Publisher<rclrs::vendor::rosgraph_msgs::msg::Clock>>,
}

impl SimulatorClock {
    pub fn new(node: rclrs::Node) -> Result<SimulatorClock> {
        // Use default QoS for clock topic (no prefix for namespace flexibility)
        let publisher_clock = node.create_publisher("clock")?;

        Ok(SimulatorClock {
            publisher_clock: Arc::new(publisher_clock),
        })
    }

    pub fn publish_clock(&self, timestamp: Option<f64>) -> Result<()> {
        let time = if let Some(sec) = timestamp {
            rclrs::vendor::builtin_interfaces::msg::Time {
                sec: sec.floor() as i32,
                nanosec: (sec.fract() * 1_000_000_000_f64) as u32,
            }
        } else {
            // If there is no timestamp, use system time
            let now = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .expect("Unable to get current time");
            rclrs::vendor::builtin_interfaces::msg::Time {
                sec: now.as_secs() as i32,
                nanosec: now.subsec_nanos(),
            }
        };

        // Wrap the time in a Clock message (ROS 2 standard for /clock topic)
        let clock_msg = rclrs::vendor::rosgraph_msgs::msg::Clock { clock: time };

        self.publisher_clock.publish(clock_msg)?;
        Ok(())
    }
}
