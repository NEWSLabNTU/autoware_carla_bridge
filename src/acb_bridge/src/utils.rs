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

/// Maps a CARLA simulation timestamp onto the `/clock` epoch Autoware runs on.
///
/// Sensor messages used to be stamped with [`ros_time_now`], read inside the CARLA callback.
/// That is the *tick*, not the measurement: `/clock` steps at the frame rate, so every
/// callback between two `/clock` messages shares a stamp and a late `/clock` makes the next
/// stamp jump by the whole gap. `gyro_odometer` derives `imu_dt` from these stamps and times
/// out at 0.2 s -- measured at up to 598 ms on a run that then failed, against 200 ms on the
/// same stack's passing run. See `docs/issues/016-*`.
///
/// Using CARLA's timestamp raw is not the answer either, for the reason [`ros_time_now`]
/// gives: it is server *uptime*, tens of thousands of seconds against a scenario `/clock`
/// near zero, and mixing the epochs made NDT reject every scan. So carry an offset between
/// the two and add it, keeping Autoware's epoch and the sensor's true spacing.
///
/// The offset is re-learned rather than fixed: `/clock` restarts with each scenario -- a
/// jump of about 31 s was measured at one run boundary -- so a value learned once would be
/// wrong for every run after the first.
///
/// # Stamps must land exactly on the `/clock` grid
///
/// A sensor stamp has to equal, to the nanosecond, the `/clock` value of the frame it was
/// taken in. NDT's pose buffer (`SmartPoseBuffer::pop_old`) drops every EKF pose stamped
/// *strictly* before the scan it just matched. When a scan came out 1 ns after the EKF pose
/// of its own frame -- `...800433` against `...800432`, from truncating CARLA's `f64` seconds
/// with `as i64` at two different ticks -- that pop emptied the buffer, the next scan found
/// `pose_buffer_.size() < 2`, and NDT warned "Couldn't interpolate pose" on alternate scans
/// (135 of 427 in one ego_drive). The graph needs `/autoware/localization` at OK for the
/// autonomous mode to count as available, so each warning that happened to be current when
/// the graph was evaluated was an MRM emergency stop -- 26 of the 27 driving MRMs across one
/// seven-run suite. So both CARLA readings are rounded to the microsecond (CARLA's step is a
/// whole number of milliseconds; the `f64` noise is far below that) and the stamp is exact.
///
/// # One reading does not decide the offset
///
/// The scenario runner ticks CARLA and *then* publishes that frame's `/clock`, and this
/// node only sees a new `/clock` when the main loop spins. So a reading pairs the newest
/// CARLA time with either that frame's `/clock` or the previous one, whichever the last
/// spin happened to catch: two values a frame apart, in proportions that vary from run to
/// run (72:28 in one, 5:95 in the next). Taking each reading as the offset made the stamps
/// flip between the two, publishing some scans twice under one stamp and skipping the
/// next (74 of 701 in one run). So the offset is the most frequent reading of the last
/// [`SimClockOffset::WINDOW`], and a tie keeps the current one: it no longer flips, and a
/// genuine change still wins within the window. A change of more than
/// [`SimClockOffset::EPOCH_JUMP_NANOS`] is a new `/clock` epoch and is taken at once.
#[derive(Clone, Debug)]
pub struct SimClockOffset {
    /// `/clock` nanoseconds minus CARLA simulation nanoseconds. `i64::MIN` means unset.
    offset_nanos: std::sync::Arc<std::sync::atomic::AtomicI64>,
    /// The most recent readings, newest last; the offset is their most frequent value.
    recent: std::sync::Arc<std::sync::Mutex<std::collections::VecDeque<i64>>>,
}

impl Default for SimClockOffset {
    fn default() -> Self {
        Self::new()
    }
}

impl SimClockOffset {
    /// Readings the most frequent value is taken over: several seconds at the main loop's
    /// 12-20 Hz.
    pub const WINDOW: usize = 100;
    /// A reading this far from the current offset starts a new epoch (a new scenario).
    /// Far above the one-frame disagreement the window settles, far below the ~31 s
    /// measured at a run boundary.
    pub const EPOCH_JUMP_NANOS: i64 = 500_000_000;

    pub fn new() -> Self {
        Self {
            offset_nanos: std::sync::Arc::new(std::sync::atomic::AtomicI64::new(i64::MIN)),
            recent: std::sync::Arc::new(std::sync::Mutex::new(
                std::collections::VecDeque::with_capacity(Self::WINDOW),
            )),
        }
    }

    /// CARLA seconds as whole microseconds, in nanoseconds. See the type documentation.
    fn carla_nanos(carla_secs: f64) -> i64 {
        (carla_secs * 1e6).round() as i64 * 1_000
    }

    /// Re-learn the offset from a `/clock` reading and a CARLA reading taken together.
    ///
    /// Called once per tick from the main loop, which is where both clocks are in hand.
    pub fn observe(&self, node: &rclrs::Node, carla_elapsed_secs: f64) {
        self.observe_nanos(node.get_clock().now().nsec, carla_elapsed_secs);
    }

    /// [`Self::observe`] with the `/clock` reading already taken.
    pub fn observe_nanos(&self, ros_nanos: i64, carla_elapsed_secs: f64) {
        if carla_elapsed_secs <= 0.0 {
            return; // no frame yet; nothing to anchor to
        }
        if ros_nanos <= 0 {
            return; // simulation clock not driven yet
        }
        let reading = ros_nanos - Self::carla_nanos(carla_elapsed_secs);
        // A poisoned lock only means another observer panicked mid-update; the readings
        // themselves are plain integers and still usable.
        let mut recent = self.recent.lock().unwrap_or_else(|e| e.into_inner());
        let current = self.offset_nanos.load(std::sync::atomic::Ordering::Relaxed);
        if current != i64::MIN && (reading - current).abs() > Self::EPOCH_JUMP_NANOS {
            recent.clear();
        }
        if recent.len() == Self::WINDOW {
            recent.pop_front();
        }
        recent.push_back(reading);
        let offset = Self::settle(&recent, current);
        self.offset_nanos
            .store(offset, std::sync::atomic::Ordering::Relaxed);
    }

    /// The most frequent reading; `current` wins a tie, then the larger value.
    fn settle(recent: &std::collections::VecDeque<i64>, current: i64) -> i64 {
        let mut counts: Vec<(i64, usize)> = Vec::new();
        for &r in recent {
            match counts.iter_mut().find(|(v, _)| *v == r) {
                Some((_, n)) => *n += 1,
                None => counts.push((r, 1)),
            }
        }
        counts
            .into_iter()
            .max_by_key(|&(v, n)| (n, v == current, v))
            .map_or(current, |(v, _)| v)
    }

    /// Stamp for a CARLA sensor timestamp, or `None` before the offset is known.
    pub fn stamp(&self, carla_timestamp_secs: f64) -> Option<builtin_interfaces::msg::Time> {
        self.stamp_nanos(carla_timestamp_secs)
            .map(nanos_to_ros_time)
    }

    /// [`Self::stamp`] as nanoseconds.
    pub fn stamp_nanos(&self, carla_timestamp_secs: f64) -> Option<i64> {
        let offset = self.offset_nanos.load(std::sync::atomic::Ordering::Relaxed);
        if offset == i64::MIN {
            return None;
        }
        Some(Self::carla_nanos(carla_timestamp_secs) + offset)
    }
}

/// Build a header stamped from the node's ROS clock. See [`ros_time_now`].
pub fn create_ros_header_from_node(node: &rclrs::Node) -> std_msgs::msg::Header {
    std_msgs::msg::Header {
        stamp: ros_time_now(node),
        frame_id: String::new(),
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

    /// CARLA's elapsed time accumulated in `f64` over a long uptime, as the server
    /// reports it: `k` steps of 0.05 s past `base`, with the rounding noise that brings.
    fn carla_secs(base: f64, k: u32) -> f64 {
        (0..k).fold(base, |t, _| t + 0.05)
    }

    /// The regression: a scan stamped even 1 ns after its frame's `/clock` value makes NDT
    /// discard that frame's EKF pose and fail the next scan. Every tick's stamp must equal
    /// the `/clock` value of that tick exactly, whichever tick the offset was learned at.
    #[test]
    fn stamps_land_exactly_on_the_clock_grid() {
        let clock0: i64 = 1_790_548_286_337_800_432;
        let base = 264_104.123_456_7;
        let offset = SimClockOffset::new();
        for k in 0..400u32 {
            let clock = clock0 + i64::from(k) * 50_000_000;
            let carla = carla_secs(base, k);
            offset.observe_nanos(clock, carla);
            // The sensor of this tick, and of the tick the next reading will cover.
            assert_eq!(offset.stamp_nanos(carla), Some(clock), "tick {k}");
            let next = carla_secs(base, k + 1);
            assert_eq!(
                offset.stamp_nanos(next),
                Some(clock + 50_000_000),
                "tick {k}+1"
            );
        }
    }

    /// A reading taken after the tick but before its `/clock` arrives is one frame low.
    /// It must not move the stamps.
    #[test]
    fn a_reading_one_frame_low_is_ignored() {
        let offset = SimClockOffset::new();
        let base = 1000.0;
        offset.observe_nanos(60_000_000_000, carla_secs(base, 0));
        // Next tick: CARLA advanced, /clock not yet.
        offset.observe_nanos(60_000_000_000, carla_secs(base, 1));
        assert_eq!(
            offset.stamp_nanos(carla_secs(base, 1)),
            Some(60_050_000_000)
        );
    }

    /// Readings split between two values a frame apart, as measured live (5% one way in
    /// one run, 28% the other way in another), must give one stamp per tick, never two.
    #[test]
    fn a_mixed_stream_of_readings_does_not_flip_the_stamps() {
        for every in [20u32, 4, 3] {
            let offset = SimClockOffset::new();
            let base = 5000.0;
            let clock0: i64 = 70_000_000_000;
            let mut stamps = Vec::new();
            for k in 0..600u32 {
                let clock = clock0 + i64::from(k) * 50_000_000;
                // The minority reading still pairs this tick with the previous /clock.
                let seen = if (k + 1) % every == 0 {
                    clock - 50_000_000
                } else {
                    clock
                };
                offset.observe_nanos(seen, carla_secs(base, k));
                stamps.push(offset.stamp_nanos(carla_secs(base, k)).unwrap());
            }
            let steps: std::collections::BTreeSet<i64> =
                stamps.windows(2).map(|w| w[1] - w[0]).collect();
            assert_eq!(steps, [50_000_000].into(), "1 in {every} readings low");
        }
    }

    /// A lasting change is followed once the low readings fill the window.
    #[test]
    fn a_lasting_change_is_followed_within_the_window() {
        let offset = SimClockOffset::new();
        let base = 1000.0;
        offset.observe_nanos(60_000_000_000, carla_secs(base, 0));
        for k in 1..=SimClockOffset::WINDOW as u32 {
            // CARLA ticks once more per frame than /clock advances.
            offset.observe_nanos(
                60_000_000_000 + i64::from(k - 1) * 50_000_000,
                carla_secs(base, k),
            );
        }
        let k = SimClockOffset::WINDOW as u32;
        assert_eq!(
            offset.stamp_nanos(carla_secs(base, k)),
            Some(60_000_000_000 + i64::from(k - 1) * 50_000_000)
        );
    }

    /// A new scenario restarts `/clock`; the new epoch applies from its first reading.
    #[test]
    fn a_new_epoch_is_taken_at_once() {
        let offset = SimClockOffset::new();
        let base = 1000.0;
        offset.observe_nanos(90_000_000_000, carla_secs(base, 0));
        offset.observe_nanos(59_000_000_000, carla_secs(base, 1));
        assert_eq!(
            offset.stamp_nanos(carla_secs(base, 1)),
            Some(59_000_000_000)
        );
    }

    #[test]
    fn no_stamp_before_the_first_reading() {
        let offset = SimClockOffset::new();
        assert_eq!(offset.stamp_nanos(12.0), None);
        offset.observe_nanos(0, 12.0); // /clock not driven yet
        assert_eq!(offset.stamp_nanos(12.0), None);
    }

    #[test]
    fn a_zero_based_server_is_unchanged() {
        let mut epoch = ClockEpoch::new();
        assert_eq!(epoch.to_sim_time(0.0), 0.0);
        assert!((epoch.to_sim_time(0.05) - 0.05).abs() < 1e-9);
    }
}
