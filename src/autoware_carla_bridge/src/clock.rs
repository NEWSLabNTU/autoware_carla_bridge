use std::{
    sync::Arc,
    time::{SystemTime, UNIX_EPOCH},
};

use crate::error::Result;

pub struct SimulatorClock {
    publisher_clock: Arc<rclrs::Publisher<builtin_interfaces::msg::Time>>,
}

impl SimulatorClock {
    pub fn new(node: rclrs::Node) -> Result<SimulatorClock> {
        // Use default QoS for clock topic
        let publisher_clock = node.create_publisher("/clock")?;

        Ok(SimulatorClock {
            publisher_clock: Arc::new(publisher_clock),
        })
    }

    pub fn publish_clock(&self, timestamp: Option<f64>) -> Result<()> {
        let time = if let Some(sec) = timestamp {
            builtin_interfaces::msg::Time {
                sec: sec.floor() as i32,
                nanosec: (sec.fract() * 1_000_000_000_f64) as u32,
            }
        } else {
            // If there is no timestamp, use system time
            let now = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .expect("Unable to get current time");
            builtin_interfaces::msg::Time {
                sec: now.as_secs() as i32,
                nanosec: now.subsec_nanos(),
            }
        };

        self.publisher_clock.publish(time)?;
        Ok(())
    }
}
