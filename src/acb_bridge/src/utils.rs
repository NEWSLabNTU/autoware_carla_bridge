use std::time::{SystemTime, UNIX_EPOCH};

pub fn is_bigendian() -> bool {
    cfg!(target_endian = "big")
}

/// Read the current simulation time from the node's ROS clock.
///
/// With `use_sim_time=true` this is driven by the domain's `/clock` topic, which is
/// the only epoch that agrees with the rest of the stack. Use this for every message
/// stamp.
///
/// # Why not CARLA's timestamps
///
/// `SensorData::timestamp()` and `WorldSnapshot::timestamp().elapsed_seconds` report
/// CARLA *server uptime* — tens of thousands of seconds on a long-running server —
/// while a scenario `/clock` starts near zero. Mixing the two makes NDT reject every
/// scan with `Validation error. reference time 9770s, target time 59s`. See
/// `docs/design/multi-instance-architecture.md` (gap 3).
pub fn ros_time_now(node: &rclrs::Node) -> builtin_interfaces::msg::Time {
    // Fetch the clock on every call rather than caching it: rclrs swaps the node's
    // clock instance when `use_sim_time` is resolved, so a Clock cloned during setup
    // can be a stale system clock.
    //
    // Read `nsec` directly instead of `Time::to_ros_msg()`. That returns rclrs's own
    // *vendored* `builtin_interfaces::msg::Time`, which is a distinct type from the
    // colcon-generated one every other message in this crate uses.
    nanos_to_ros_time(node.get_clock().now().nsec)
}

/// Split nanoseconds into a ROS `Time`.
fn nanos_to_ros_time(nanos: i64) -> builtin_interfaces::msg::Time {
    // ROS time is unsigned. A negative reading means the simulation clock has not been
    // driven yet, which is normal for the first few callbacks after startup.
    let nanos = nanos.max(0);
    builtin_interfaces::msg::Time {
        sec: (nanos / 1_000_000_000) as i32,
        nanosec: (nanos % 1_000_000_000) as u32,
    }
}

/// Same as [`ros_time_now`], as fractional seconds.
///
/// Convenience for the interfaces that still take an `f64` timestamp. `i64`
/// nanoseconds convert exactly for any value below 2^53 ns (~104 days).
pub fn ros_time_now_secs(node: &rclrs::Node) -> f64 {
    node.get_clock().now().nsec as f64 / 1e9
}

/// Build a header stamped from the node's ROS clock. See [`ros_time_now`].
pub fn create_ros_header_from_node(node: &rclrs::Node) -> std_msgs::msg::Header {
    std_msgs::msg::Header {
        stamp: ros_time_now(node),
        frame_id: String::new(),
    }
}

/// Build a header from an explicit epoch-seconds value.
///
/// **The caller owns the epoch.** Pass a value already on the same timeline as the
/// domain's `/clock`. Never pass a raw CARLA timestamp — prefer
/// [`create_ros_header_from_node`] unless you have a specific reason not to.
pub fn create_ros_header_from_epoch_seconds(timestamp: Option<f64>) -> std_msgs::msg::Header {
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

/// Converts CARLA server uptime into scenario-relative simulation time.
///
/// CARLA's `elapsed_seconds` counts from server start, not from scenario start, so a
/// bridge that publishes `/clock` must subtract the value it saw on its first tick.
/// Without this, Autoware receives a clock in the tens of thousands of seconds while
/// every other participant starts near zero.
///
/// Only used when this bridge owns `/clock` (see `publish_clock`). In the scenario
/// ego's domain SSv2 owns `/clock` and this offset is never applied.
#[derive(Debug, Default)]
pub struct ClockEpoch {
    epoch: Option<f64>,
}

impl ClockEpoch {
    pub fn new() -> Self {
        Self { epoch: None }
    }

    /// Map raw CARLA elapsed seconds to scenario-relative seconds.
    ///
    /// The first call establishes the epoch and returns 0.0.
    // Not a `to_*` conversion of `self` -- it converts its argument and mutates the
    // epoch, so clippy's `to_*`-takes-`&self` convention does not apply.
    #[allow(clippy::wrong_self_convention)]
    pub fn to_sim_time(&mut self, carla_elapsed_seconds: f64) -> f64 {
        let epoch = *self.epoch.get_or_insert(carla_elapsed_seconds);
        carla_elapsed_seconds - epoch
    }

    /// Forget the epoch, so the next call re-establishes it.
    ///
    /// Called when the CARLA connection is re-established: a restarted server resets
    /// `elapsed_seconds`, and keeping the old epoch would drive the clock negative.
    pub fn reset(&mut self) {
        self.epoch = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nanos_split_into_sec_and_nanosec() {
        let t = nanos_to_ros_time(1_500_000_000);
        assert_eq!(t.sec, 1);
        assert_eq!(t.nanosec, 500_000_000);
    }

    #[test]
    fn whole_seconds_have_no_remainder() {
        let t = nanos_to_ros_time(59_000_000_000);
        assert_eq!(t.sec, 59);
        assert_eq!(t.nanosec, 0);
    }

    /// ROS time is unsigned; an undriven sim clock can read negative.
    #[test]
    fn negative_nanos_clamp_to_zero() {
        let t = nanos_to_ros_time(-1);
        assert_eq!(t.sec, 0);
        assert_eq!(t.nanosec, 0);
    }

    /// Regression guard for gap 3: a bridge that owns `/clock` must publish
    /// scenario-relative time, not CARLA server uptime.
    #[test]
    fn first_tick_establishes_the_epoch() {
        let mut epoch = ClockEpoch::new();
        assert_eq!(epoch.to_sim_time(9770.0), 0.0);
    }

    #[test]
    fn later_ticks_are_relative_to_the_first() {
        let mut epoch = ClockEpoch::new();
        epoch.to_sim_time(9770.0);
        assert!((epoch.to_sim_time(9770.05) - 0.05).abs() < 1e-9);
        assert!((epoch.to_sim_time(9829.0) - 59.0).abs() < 1e-9);
    }

    /// A CARLA restart rewinds `elapsed_seconds`; without a reset the clock would go
    /// negative and Autoware would log "jump back in time".
    #[test]
    fn reset_re_establishes_the_epoch_after_reconnect() {
        let mut epoch = ClockEpoch::new();
        epoch.to_sim_time(9770.0);
        epoch.to_sim_time(9829.0);

        epoch.reset();

        assert_eq!(epoch.to_sim_time(12.0), 0.0);
        assert!((epoch.to_sim_time(13.5) - 1.5).abs() < 1e-9);
    }

    #[test]
    fn a_zero_based_server_is_unchanged() {
        let mut epoch = ClockEpoch::new();
        assert_eq!(epoch.to_sim_time(0.0), 0.0);
        assert!((epoch.to_sim_time(0.05) - 0.05).abs() < 1e-9);
    }
}
