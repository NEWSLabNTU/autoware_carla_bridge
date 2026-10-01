pub fn is_bigendian() -> bool {
    cfg!(target_endian = "big")
}

/// The node's ROS clock, as fractional seconds.
///
/// Not for stamps: every message this bridge publishes is stamped with the CARLA frame time
/// it was computed from, through `clock::SimClock::stamp` (roadmap 015 in
/// carla-scenario-bridge). This is for comparing against stamps Autoware sent us, which
/// are in the same time base once `/clock` is ours.
pub fn ros_time_now_secs(node: &rclrs::Node) -> f64 {
    node.get_clock().now().nsec as f64 / 1e9
}
