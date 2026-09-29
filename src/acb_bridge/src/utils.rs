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
/// # A stamp must never be later than its frame's `/clock`
///
/// NDT's pose buffer (`SmartPoseBuffer::pop_old`) drops every EKF pose stamped *strictly*
/// before the scan it just matched, and the EKF stamps its poses with `/clock`. A scan stamped
/// even 1 ns after the `/clock` of its own frame therefore drops that frame's pose, and the
/// next scan -- which arrives before the EKF has published another -- finds
/// `pose_buffer_.size() < 2`: "Couldn't interpolate pose". The diagnostic graph requires
/// `/autoware/localization` at OK for the autonomous mode (Autoware counts a mode available
/// only at OK, so a WARN is enough), and every such warning the graph caught was an MRM
/// emergency stop. A stamp slightly *early* is harmless: the pose it interpolates is the
/// frame's own to within a microsecond, and nothing is popped that is still needed.
///
/// Exact equality is not attainable, for two measured reasons:
///
/// - CARLA steps its clock by `fixed_delta_seconds` held as an `f32`: 0.05 becomes
///   0.050000000745 s, so `elapsed_seconds` runs 0.745 ns per tick ahead of the 50 ms grid
///   SSv2's `/clock` follows. Rounded to the microsecond (the previous scheme), the reading
///   gained a whole microsecond every ~1340 ticks; the most-frequent-reading window then
///   kept the old offset for half a window, ~2.5 s, stamping every scan 1 us *late* -- a
///   2.5 s "Couldn't interpolate pose" episode and an emergency stop, 16 of 16 MRM onsets in
///   one session (roadmap 014, 2026-09-29).
/// - SSv2 computes `/clock` in floating point: its steps are 50 ms +- 1 ns, and the phase
///   of the grid wobbles by a nanosecond from frame to frame.
///
/// So a stamp is placed on the grid of the newest `/clock` reading, one CARLA tick apart
/// (snapping removes CARLA's drift entirely), and then [`SimClockOffset::MARGIN_NANOS`]
/// before it, which covers the nanosecond wobble with room to spare.
///
/// # One reading does not decide the offset
///
/// The scenario runner ticks CARLA and *then* publishes that frame's `/clock`, and this
/// node only sees a new `/clock` when the main loop spins. So a reading pairs the newest
/// CARLA time with either that frame's `/clock` or the previous one, whichever the last
/// spin happened to catch: two values a frame apart, in proportions that vary from run to
/// run (72:28 in one, 5:95 in the next). Taking each reading as the offset made the stamps
/// flip between the two, publishing some scans twice under one stamp and skipping the
/// next (74 of 701 in one run). So the offset comes from the largest cluster of the last
/// [`SimClockOffset::WINDOW`] readings, readings within a quarter tick counting as one
/// value (the drift above would otherwise split the majority in two and hand the window to
/// the minority, stamping a scan a whole frame ahead); a tie keeps the current cluster. A
/// change of more than [`SimClockOffset::EPOCH_JUMP_NANOS`] is a new `/clock` epoch and is
/// taken at once.
#[derive(Clone, Debug)]
pub struct SimClockOffset {
    /// `/clock` nanoseconds minus CARLA simulation nanoseconds. `i64::MIN` means unset.
    offset_nanos: std::sync::Arc<std::sync::atomic::AtomicI64>,
    /// The newest `/clock` reading: the phase of the grid stamps are placed on.
    clock_nanos: std::sync::Arc<std::sync::atomic::AtomicI64>,
    /// One CARLA tick in nanoseconds, the grid's spacing; 0 until a tick length is known.
    step_nanos: std::sync::Arc<std::sync::atomic::AtomicI64>,
    /// The most recent readings, newest last.
    recent: std::sync::Arc<std::sync::Mutex<std::collections::VecDeque<i64>>>,
}

impl Default for SimClockOffset {
    fn default() -> Self {
        Self::new()
    }
}

impl SimClockOffset {
    /// Readings the offset is taken over: several seconds at the main loop's 12-20 Hz.
    pub const WINDOW: usize = 100;
    /// A reading this far from the current offset starts a new epoch (a new scenario).
    /// Far above the one-frame disagreement the window settles, far below the ~31 s
    /// measured at a run boundary.
    pub const EPOCH_JUMP_NANOS: i64 = 500_000_000;
    /// How far before its frame's `/clock` a stamp is placed. `/clock` wobbles by 1 ns;
    /// a microsecond is a thousand times that and still moves a 20 m/s ego by 20 um.
    pub const MARGIN_NANOS: i64 = 1_000;
    /// Cluster tolerance before a tick length is known.
    const DEFAULT_TOLERANCE_NANOS: i64 = 10_000_000;

    pub fn new() -> Self {
        Self {
            offset_nanos: std::sync::Arc::new(std::sync::atomic::AtomicI64::new(i64::MIN)),
            clock_nanos: std::sync::Arc::new(std::sync::atomic::AtomicI64::new(i64::MIN)),
            step_nanos: std::sync::Arc::new(std::sync::atomic::AtomicI64::new(0)),
            recent: std::sync::Arc::new(std::sync::Mutex::new(
                std::collections::VecDeque::with_capacity(Self::WINDOW),
            )),
        }
    }

    /// CARLA seconds in nanoseconds. Deliberately unrounded: the grid snap in
    /// [`Self::stamp_nanos`] is what makes stamps exact, not rounding here.
    fn carla_nanos(carla_secs: f64) -> i64 {
        (carla_secs * 1e9).round() as i64
    }

    /// Re-learn the offset from a `/clock` reading and a CARLA reading taken together.
    ///
    /// Called once per tick from the main loop, which is where both clocks are in hand.
    /// `tick_secs` is CARLA's `delta_seconds`, the spacing of the grid stamps land on.
    pub fn observe(&self, node: &rclrs::Node, carla_elapsed_secs: f64, tick_secs: f64) {
        self.observe_nanos(node.get_clock().now().nsec, carla_elapsed_secs, tick_secs);
    }

    /// [`Self::observe`] with the `/clock` reading already taken.
    pub fn observe_nanos(&self, ros_nanos: i64, carla_elapsed_secs: f64, tick_secs: f64) {
        if carla_elapsed_secs <= 0.0 {
            return; // no frame yet; nothing to anchor to
        }
        if ros_nanos <= 0 {
            return; // simulation clock not driven yet
        }
        use std::sync::atomic::Ordering::Relaxed;
        // CARLA's tick is a whole number of microseconds; its f32 excess is not a real step.
        let step = if tick_secs > 0.0 {
            (tick_secs * 1e6).round() as i64 * 1_000
        } else {
            self.step_nanos.load(Relaxed)
        };
        let reading = ros_nanos - Self::carla_nanos(carla_elapsed_secs);
        // A poisoned lock only means another observer panicked mid-update; the readings
        // themselves are plain integers and still usable.
        let mut recent = self.recent.lock().unwrap_or_else(|e| e.into_inner());
        let current = self.offset_nanos.load(Relaxed);
        if current != i64::MIN && (reading - current).abs() > Self::EPOCH_JUMP_NANOS {
            recent.clear();
        }
        if recent.len() == Self::WINDOW {
            recent.pop_front();
        }
        recent.push_back(reading);
        let tolerance = if step > 0 {
            step / 4
        } else {
            Self::DEFAULT_TOLERANCE_NANOS
        };
        let offset = Self::settle(&recent, current, tolerance);
        self.step_nanos.store(step, Relaxed);
        self.clock_nanos.store(ros_nanos, Relaxed);
        self.offset_nanos.store(offset, Relaxed);
    }

    /// The newest reading of the largest cluster (readings within `tolerance` of one
    /// another); a tie goes to the cluster holding `current`, then to the newer reading.
    fn settle(recent: &std::collections::VecDeque<i64>, current: i64, tolerance: i64) -> i64 {
        let near = |a: i64, b: i64| (a - b).abs() <= tolerance;
        let best = recent
            .iter()
            .enumerate()
            .max_by_key(|&(i, &r)| {
                let n = recent.iter().filter(|&&x| near(x, r)).count();
                (n, current != i64::MIN && near(r, current), i)
            })
            .map(|(_, &r)| r);
        match best {
            Some(center) => recent
                .iter()
                .rev()
                .copied()
                .find(|&x| near(x, center))
                .unwrap_or(center),
            None => current,
        }
    }

    /// Stamp for a CARLA sensor timestamp, or `None` before the offset is known.
    pub fn stamp(&self, carla_timestamp_secs: f64) -> Option<builtin_interfaces::msg::Time> {
        self.stamp_nanos(carla_timestamp_secs)
            .map(nanos_to_ros_time)
    }

    /// [`Self::stamp`] as nanoseconds: on the `/clock` grid, [`Self::MARGIN_NANOS`] early.
    pub fn stamp_nanos(&self, carla_timestamp_secs: f64) -> Option<i64> {
        use std::sync::atomic::Ordering::Relaxed;
        let offset = self.offset_nanos.load(Relaxed);
        if offset == i64::MIN {
            return None;
        }
        let raw = Self::carla_nanos(carla_timestamp_secs) + offset;
        let step = self.step_nanos.load(Relaxed);
        let clock = self.clock_nanos.load(Relaxed);
        let on_grid = if step > 0 && clock != i64::MIN {
            let d = raw - clock;
            // Nearest grid point; `raw` is within microseconds of one.
            clock + (d + d.signum() * step / 2) / step * step
        } else {
            raw
        };
        Some(on_grid - Self::MARGIN_NANOS)
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

    const TICK: f64 = 0.05;
    const STEP: i64 = 50_000_000;
    const M: i64 = SimClockOffset::MARGIN_NANOS;

    /// CARLA's elapsed time as the server reports it: `k` ticks past `base`, each the
    /// `f32` 0.05 (0.050000000745 s) accumulated in `f64` -- 0.745 ns per tick ahead of the
    /// 50 ms grid, a whole microsecond every ~1340 ticks.
    fn carla_secs(base: f64, k: u32) -> f64 {
        (0..k).fold(base, |t, _| t + f64::from(0.05f32))
    }

    /// SSv2's `/clock` for tick `k`: a 50 ms grid whose phase wobbles by a nanosecond, as
    /// measured (steps of 49999999, 50000000 and 50000001 ns).
    fn clock_of(clock0: i64, k: u32) -> i64 {
        clock0 + i64::from(k) * STEP + [0, -1, 0, 1][k as usize % 4]
    }

    /// The regression: a scan stamped even 1 ns after its frame's `/clock` makes NDT drop
    /// that frame's EKF pose and fail the next scan. Over 5000 ticks -- CARLA's drift
    /// crosses three microsecond boundaries, which used to stamp ~50 ticks each a
    /// microsecond late -- every stamp must sit just before the `/clock` of its tick.
    #[test]
    fn stamps_never_pass_the_clock_of_their_tick() {
        let clock0: i64 = 1_790_548_286_337_800_432;
        let base = 264_104.123_456_7;
        let offset = SimClockOffset::new();
        for k in 0..5000u32 {
            let clock = clock_of(clock0, k);
            offset.observe_nanos(clock, carla_secs(base, k), TICK);
            // The sensor of this tick, and of the tick the next reading will cover.
            for (tick, truth) in [(k, clock), (k + 1, clock_of(clock0, k + 1))] {
                let s = offset.stamp_nanos(carla_secs(base, tick)).unwrap();
                assert!(s < truth, "tick {tick}: stamp {s} not before clock {truth}");
                assert!(
                    truth - s <= M + 2,
                    "tick {tick}: stamp {s} too early for {truth}"
                );
            }
        }
    }

    /// A reading taken after the tick but before its `/clock` arrives is one frame low.
    /// It must not move the stamps.
    #[test]
    fn a_reading_one_frame_low_is_ignored() {
        let offset = SimClockOffset::new();
        let base = 1000.0;
        offset.observe_nanos(60_000_000_000, carla_secs(base, 0), TICK);
        // Next tick: CARLA advanced, /clock not yet.
        offset.observe_nanos(60_000_000_000, carla_secs(base, 1), TICK);
        assert_eq!(
            offset.stamp_nanos(carla_secs(base, 1)),
            Some(60_050_000_000 - M)
        );
    }

    /// Readings split between two values a frame apart, as measured live (5% one way in
    /// one run, 28% the other way in another), must give one stamp per tick, never two --
    /// also while CARLA's drift carries the majority across microsecond boundaries, which
    /// used to split it in two and let the minority win the window.
    #[test]
    fn a_mixed_stream_of_readings_does_not_flip_the_stamps() {
        for every in [20u32, 4, 3] {
            let offset = SimClockOffset::new();
            let base = 5_000.123_456_4;
            let clock0: i64 = 70_000_000_000;
            let mut stamps = Vec::new();
            for k in 0..4000u32 {
                let clock = clock0 + i64::from(k) * STEP;
                // The minority reading still pairs this tick with the previous /clock.
                let seen = if (k + 1) % every == 0 {
                    clock - STEP
                } else {
                    clock
                };
                offset.observe_nanos(seen, carla_secs(base, k), TICK);
                let s = offset.stamp_nanos(carla_secs(base, k)).unwrap();
                assert!(s < clock, "1 in {every}, tick {k}");
                stamps.push(s);
            }
            let steps: std::collections::BTreeSet<i64> =
                stamps.windows(2).map(|w| w[1] - w[0]).collect();
            assert_eq!(steps, [STEP].into(), "1 in {every} readings low");
        }
    }

    /// A lasting change is followed once the new readings are the majority of the window.
    #[test]
    fn a_lasting_change_is_followed_within_the_window() {
        let offset = SimClockOffset::new();
        let base = 1000.0;
        offset.observe_nanos(60_000_000_000, carla_secs(base, 0), TICK);
        for k in 1..=SimClockOffset::WINDOW as u32 {
            // CARLA ticks once more per frame than /clock advances.
            offset.observe_nanos(
                60_000_000_000 + i64::from(k - 1) * STEP,
                carla_secs(base, k),
                TICK,
            );
        }
        let k = SimClockOffset::WINDOW as u32;
        assert_eq!(
            offset.stamp_nanos(carla_secs(base, k)),
            Some(60_000_000_000 + i64::from(k - 1) * STEP - M)
        );
    }

    /// A new scenario restarts `/clock`; the new epoch applies from its first reading.
    #[test]
    fn a_new_epoch_is_taken_at_once() {
        let offset = SimClockOffset::new();
        let base = 1000.0;
        offset.observe_nanos(90_000_000_000, carla_secs(base, 0), TICK);
        offset.observe_nanos(59_000_000_000, carla_secs(base, 1), TICK);
        assert_eq!(
            offset.stamp_nanos(carla_secs(base, 1)),
            Some(59_000_000_000 - M)
        );
    }

    #[test]
    fn no_stamp_before_the_first_reading() {
        let offset = SimClockOffset::new();
        assert_eq!(offset.stamp_nanos(12.0), None);
        offset.observe_nanos(0, 12.0, TICK); // /clock not driven yet
        assert_eq!(offset.stamp_nanos(12.0), None);
    }

    #[test]
    fn a_zero_based_server_is_unchanged() {
        let mut epoch = ClockEpoch::new();
        assert_eq!(epoch.to_sim_time(0.0), 0.0);
        assert!((epoch.to_sim_time(0.05) - 0.05).abs() < 1e-9);
    }
}
