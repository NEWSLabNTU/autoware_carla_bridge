use std::time::{SystemTime, UNIX_EPOCH};

use carla::client::{Client, World};

pub fn is_bigendian() -> bool {
    cfg!(target_endian = "big")
}

/// Load a CARLA world, using efficient loading if available.
///
/// For CARLA 0.9.16+, uses `load_world_if_different()` which only reloads
/// if the map is actually different, saving 5-10 seconds on redundant loads.
/// For older versions, falls back to standard `load_world()`.
pub fn load_world_smart(client: &Client, map_name: &str) -> World {
    log::info!("Loading map: {}", map_name);

    #[cfg(carla_0916)]
    {
        log::debug!("Using efficient load_world_if_different (CARLA 0.9.16+)");
        client.load_world_if_different(map_name)
    }

    #[cfg(not(carla_0916))]
    {
        log::debug!("Using standard load_world (CARLA < 0.9.16)");
        client.load_world(map_name)
    }
}

pub fn create_ros_header(timestamp: Option<f64>) -> std_msgs::msg::Header {
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
    std_msgs::msg::Header {
        stamp: time,
        frame_id: "".to_string(),
    }
}
