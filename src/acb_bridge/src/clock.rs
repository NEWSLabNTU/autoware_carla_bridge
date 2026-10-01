//! Simulation time, taken from CARLA alone.
//!
//! `sim_time = elapsed_seconds + epoch`. `elapsed_seconds` is the frame time CARLA puts in
//! every `WorldSnapshot` and every `SensorData`; the epoch is 0 until the world is reloaded,
//! and a reload is a pause, not a rewind (carla-scenario-bridge roadmap 015, "Simulation time
//! is CARLA's elapsed_seconds plus an episode epoch"). Every stamp this bridge publishes and
//! every `/clock` message goes through [`SimClock::stamp`], so a sensor sample, the odometry
//! of its frame and that frame's `/clock` carry the same nanosecond value: there is nothing
//! to reconcile, and no learned offset, margin or grid snap.
//!
//! `/clock` comes from an `on_tick` subscription of its own, on a client of its own
//! ([`spawn_clock_source`]), established when CARLA is reachable -- not when a hero is found
//! -- so it runs between scenarios (wall-paced, CARLA free-runs asynchronously) and during
//! them (one step per frame the scenario ticks). It depends on CARLA and nothing else.
//!
//! # The episode rule
//!
//! CARLA restarts `elapsed_seconds` at 0 for every episode (`load_world`). Whoever sees the
//! episode change sets `epoch += E_last + Δ_last` -- the old episode's last frame time and
//! step, from the last tick received -- in integer nanoseconds, each term rounded on its
//! own: `epoch_ns += nanos(E_last) + nanos(Δ_last)` with `nanos(s) = round(s * 1e9)`,
//! rounding half away from zero (Rust's `f64::round`, C's `std::round`; *not* Python's
//! `round`, which rounds half to even and differs by 1 ns on ~1.4% of CARLA frames). The
//! scenario bridge applies the same rule to the same frame stream, so the two agree
//! bit-exactly without talking to each other.
//!
//! A reconnect to a server whose previous last frame this bridge may not have seen (CARLA
//! restarted) cannot apply the rule. It continues instead from the last `/clock` it
//! published plus one step, which keeps the clock monotonic, and says that the scenario
//! bridge's epoch will not match until the stack is restarted.

use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    time::{Duration, Instant},
};

use crate::error::Result;

/// Seconds to integer nanoseconds: `round(secs * 1e9)`, half away from zero (`f64::round`).
/// The one conversion every stamp and `/clock` value goes through, which is what makes them
/// bit-exact with one another. carla-scenario-bridge's `episode_clock::nanos` must stay
/// identical (it reports the same integer to SSv2); both carry the same table of cases.
pub fn nanos(secs: f64) -> i64 {
    (secs * 1e9).round() as i64
}

/// Split nanoseconds into a ROS `Time`. ROS time is unsigned; negative clamps to zero.
pub fn nanos_to_time(nanos: i64) -> builtin_interfaces::msg::Time {
    let nanos = nanos.max(0);
    builtin_interfaces::msg::Time {
        sec: (nanos / 1_000_000_000) as i32,
        nanosec: (nanos % 1_000_000_000) as u32,
    }
}

/// `/clock` is refused below this much simulation time. ROS time near zero makes
/// `now() - duration` negative in nodes that assume an epoch (tf2's cache, some Autoware
/// monitors); a fresh CARLA reaches this long before an ego stack is up (roadmap 015).
pub const MIN_SIM_TIME_NS: i64 = 60_000_000_000;

/// At most one `/clock` per this interval: 100 Hz. CARLA running asynchronously with
/// nothing to render can tick far faster than anything needs a clock.
pub const MIN_PUBLISH_INTERVAL: Duration = Duration::from_millis(10);

/// One CARLA frame, as the clock needs it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tick {
    /// Episode id (`WorldSnapshot::id`, which libcarla takes from the episode).
    pub episode: u64,
    pub elapsed_ns: i64,
    /// `delta_seconds`: the step that led to this frame.
    pub delta_ns: i64,
}

impl Tick {
    pub fn from_snapshot(snapshot: &carla::client::WorldSnapshot) -> Self {
        let ts = snapshot.timestamp();
        Self {
            episode: snapshot.id(),
            elapsed_ns: nanos(ts.elapsed_seconds),
            delta_ns: nanos(ts.delta_seconds),
        }
    }
}

/// An epoch change, for the log.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EpochChange {
    /// A new episode while this bridge was watching: the rule applied.
    Episode { old: i64, new: i64, sim_ns: i64 },
    /// A new episode after a reconnect: continued from the last published `/clock`.
    Reconnect { old: i64, new: i64, sim_ns: i64 },
}

/// What one frame produced.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct FrameOutcome {
    pub change: Option<EpochChange>,
    /// The `/clock` value to publish now, if any.
    pub publish_ns: Option<i64>,
    /// First frame refused by the uptime gate (log it once).
    pub refused_ns: Option<i64>,
}

/// The clock's state: pure, so the rules are testable without CARLA or ROS.
#[derive(Debug, Default)]
pub struct EpisodeClock {
    epoch_ns: i64,
    /// The last frame received.
    last: Option<Tick>,
    last_published_ns: Option<i64>,
    last_publish_at: Option<Instant>,
    /// Newest value held back by the rate cap.
    pending_ns: Option<i64>,
    /// The next frame comes over a fresh connection.
    reconnected: bool,
    gate_reported: bool,
}

impl EpisodeClock {
    pub fn new() -> Self {
        Self::default()
    }

    /// Simulation time of a CARLA frame time, in nanoseconds.
    pub fn sim_ns(&self, elapsed_secs: f64) -> i64 {
        nanos(elapsed_secs) + self.epoch_ns
    }

    /// The next frame arrives over a new connection, possibly to a restarted server.
    pub fn mark_reconnect(&mut self) {
        self.reconnected = true;
    }

    /// Track the episode; apply the epoch rule on a change.
    fn observe(&mut self, tick: Tick) -> Option<EpochChange> {
        let reconnected = std::mem::take(&mut self.reconnected);
        let change = match self.last {
            Some(last) if tick.episode != last.episode || tick.elapsed_ns < last.elapsed_ns => {
                let old = self.epoch_ns;
                if reconnected {
                    // The old episode's last frame may never have reached us.
                    let base = self
                        .last_published_ns
                        .unwrap_or(last.elapsed_ns + old)
                        .max(last.elapsed_ns + old);
                    let step = if tick.delta_ns > 0 {
                        tick.delta_ns
                    } else {
                        last.delta_ns
                    };
                    self.epoch_ns = base + step - tick.elapsed_ns;
                    Some(EpochChange::Reconnect {
                        old,
                        new: self.epoch_ns,
                        sim_ns: tick.elapsed_ns + self.epoch_ns,
                    })
                } else {
                    self.epoch_ns += last.elapsed_ns + last.delta_ns;
                    Some(EpochChange::Episode {
                        old,
                        new: self.epoch_ns,
                        sim_ns: tick.elapsed_ns + self.epoch_ns,
                    })
                }
            }
            _ => None,
        };
        self.last = Some(tick);
        change
    }

    /// One frame from the `on_tick` subscription.
    pub fn on_frame(&mut self, tick: Tick, now: Instant) -> FrameOutcome {
        let change = self.observe(tick);
        let sim = tick.elapsed_ns + self.epoch_ns;
        let mut out = FrameOutcome {
            change,
            ..Default::default()
        };
        if sim < MIN_SIM_TIME_NS {
            self.pending_ns = None;
            if !self.gate_reported {
                self.gate_reported = true;
                out.refused_ns = Some(sim);
            }
            return out;
        }
        if self.last_published_ns.is_some_and(|p| sim <= p) {
            return out; // never backwards, never twice
        }
        if self
            .last_publish_at
            .is_some_and(|at| now.saturating_duration_since(at) < MIN_PUBLISH_INTERVAL)
        {
            self.pending_ns = Some(sim);
            return out;
        }
        out.publish_ns = Some(self.published(sim, now));
        out
    }

    /// A value the rate cap held back, once the cap allows it. Called between frames, so a
    /// paused simulation's last frame still reaches `/clock`.
    pub fn flush(&mut self, now: Instant) -> Option<i64> {
        let sim = self.pending_ns?;
        if self
            .last_publish_at
            .is_some_and(|at| now.saturating_duration_since(at) < MIN_PUBLISH_INTERVAL)
        {
            return None;
        }
        self.pending_ns = None;
        if self.last_published_ns.is_some_and(|p| sim <= p) {
            return None;
        }
        Some(self.published(sim, now))
    }

    fn published(&mut self, sim: i64, now: Instant) -> i64 {
        self.pending_ns = None;
        self.last_published_ns = Some(sim);
        self.last_publish_at = Some(now);
        sim
    }
}

/// The shared clock: the epoch every stamp uses, and the `/clock` publisher. Cheap to clone.
#[derive(Clone)]
pub struct SimClock {
    state: Arc<Mutex<EpisodeClock>>,
    publisher: Arc<rclrs::Publisher<rclrs::vendor::rosgraph_msgs::msg::Clock>>,
    /// `publish_clock`: an emergency off. Stamps still follow the epoch when it is false.
    publish: bool,
    last_frame_at: Arc<Mutex<Option<Instant>>>,
}

impl SimClock {
    pub fn new(node: &rclrs::Node, publish: bool) -> Result<Self> {
        Ok(Self {
            state: Arc::new(Mutex::new(EpisodeClock::new())),
            publisher: Arc::new(node.create_publisher("clock")?),
            publish,
            last_frame_at: Arc::new(Mutex::new(None)),
        })
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, EpisodeClock> {
        self.state.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// The stamp for a CARLA frame time (`SensorData::timestamp()` or a snapshot's
    /// `elapsed_seconds`): bit-exact with the `/clock` published for that frame.
    pub fn stamp(&self, elapsed_secs: f64) -> builtin_interfaces::msg::Time {
        nanos_to_time(self.lock().sim_ns(elapsed_secs))
    }

    pub fn mark_reconnect(&self) {
        self.lock().mark_reconnect();
    }

    fn publish_ns(&self, sim_ns: i64) {
        if !self.publish {
            return;
        }
        let t = nanos_to_time(sim_ns);
        let msg = rclrs::vendor::rosgraph_msgs::msg::Clock {
            clock: rclrs::vendor::builtin_interfaces::msg::Time {
                sec: t.sec,
                nanosec: t.nanosec,
            },
        };
        if let Err(e) = self.publisher.publish(msg) {
            tracing::warn!("Failed to publish /clock: {e}");
        }
    }

    /// Called from CARLA's callback thread for every frame.
    pub fn on_frame(&self, tick: Tick) {
        let now = Instant::now();
        *self.last_frame_at.lock().unwrap_or_else(|e| e.into_inner()) = Some(now);
        let out = self.lock().on_frame(tick, now);
        match out.change {
            Some(EpochChange::Episode { old, new, sim_ns }) => tracing::info!(
                "Episode change: epoch {:.9} -> {:.9}, sim_time continues at {:.9}",
                old as f64 / 1e9,
                new as f64 / 1e9,
                sim_ns as f64 / 1e9
            ),
            Some(EpochChange::Reconnect { old, new, sim_ns }) => tracing::warn!(
                "Episode change: epoch {:.9} -> {:.9}, sim_time continues at {:.9} -- after a \
                 reconnect, so the previous episode's last frame may not have been seen: \
                 continued from the last /clock published plus one step. A running \
                 carla_scenario_bridge applies the episode rule to the frames it saw and its \
                 epoch may differ from this one until the ego stack is restarted",
                old as f64 / 1e9,
                new as f64 / 1e9,
                sim_ns as f64 / 1e9
            ),
            None => {}
        }
        if let Some(sim) = out.refused_ns {
            tracing::info!(
                "Not publishing /clock yet: simulation time is {:.3} s, below {} s. ROS time \
                 this close to zero makes `now() - duration` negative in nodes that assume an \
                 epoch; /clock starts once CARLA has been up for {} s",
                sim as f64 / 1e9,
                MIN_SIM_TIME_NS / 1_000_000_000,
                MIN_SIM_TIME_NS / 1_000_000_000
            );
        }
        if let Some(sim) = out.publish_ns {
            self.publish_ns(sim);
        }
    }

    /// Publish a value the rate cap held back, if it is due.
    pub fn flush(&self) {
        let due = self.lock().flush(Instant::now());
        if let Some(sim) = due {
            self.publish_ns(sim);
        }
    }

    fn since_last_frame(&self) -> Option<Duration> {
        self.last_frame_at
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .map(|at| at.elapsed())
    }
}

/// How often the clock thread wakes: flushes the rate cap, watches the subscription.
const CLOCK_POLL: Duration = Duration::from_millis(20);
/// No frame for this long: ask whether the world was replaced or the server is gone.
const CLOCK_STALL_CHECK: Duration = Duration::from_secs(2);
/// Consecutive failed checks before the connection is rebuilt.
const CLOCK_MAX_FAILURES: u32 = 3;

/// Run `/clock` on a CARLA client of its own, from connect, for the life of the node.
///
/// Its own client, because the session client is dropped and rebuilt after every despawn
/// to shed the sensor streams the scenario runner destroyed (`SessionExit::VehicleLost`),
/// and the clock must not stop for that. It never ticks the world.
pub fn spawn_clock_source(
    address: String,
    port: u16,
    clock: SimClock,
    running: Arc<AtomicBool>,
) -> std::thread::JoinHandle<()> {
    std::thread::Builder::new()
        .name("acb-clock".into())
        .spawn(move || {
            let mut connected_before = false;
            while running.load(Ordering::SeqCst) {
                let client = match carla::client::Client::connect(&address, port, None) {
                    Ok(mut c) => {
                        if let Err(e) = c.set_timeout(Duration::from_secs(10)) {
                            tracing::debug!("clock: set_timeout failed: {e}");
                        }
                        c
                    }
                    Err(e) => {
                        tracing::debug!("clock: CARLA not reachable ({e}); retrying");
                        std::thread::sleep(Duration::from_secs(2));
                        continue;
                    }
                };
                if connected_before {
                    clock.mark_reconnect();
                }
                if follow_frames(&client, &clock, &running) {
                    connected_before = true;
                }
            }
        })
        .expect("spawn the clock thread")
}

/// Subscribe and keep the subscription on the current world. Returns when the connection
/// looks dead (or on shutdown); `true` if a subscription was ever established.
fn follow_frames(client: &carla::client::Client, clock: &SimClock, running: &AtomicBool) -> bool {
    let subscribe =
        |clock: &SimClock| -> Option<(carla::client::World, carla::client::OnTickId, u64)> {
            let mut world = client.world().ok()?;
            let id = world.id().ok()?;
            let sink = clock.clone();
            let cb = world
                .on_tick(move |snapshot| sink.on_frame(Tick::from_snapshot(&snapshot)))
                .ok()?;
            Some((world, cb, id))
        };
    let Some(mut sub) = subscribe(clock) else {
        std::thread::sleep(Duration::from_secs(2));
        return false;
    };
    tracing::info!("/clock follows CARLA frames (episode {})", sub.2);
    let mut failures = 0u32;
    let mut last_check = Instant::now();
    while running.load(Ordering::SeqCst) {
        std::thread::sleep(CLOCK_POLL);
        clock.flush();
        let quiet = clock
            .since_last_frame()
            .is_none_or(|d| d >= CLOCK_STALL_CHECK);
        if !quiet || last_check.elapsed() < CLOCK_STALL_CHECK {
            continue;
        }
        last_check = Instant::now();
        // Nothing is ticking, or our subscription is on a world that was replaced.
        match client.world().and_then(|w| w.id().map(|id| (w, id))) {
            Ok((_, id)) if id == sub.2 => failures = 0,
            Ok(_) => {
                tracing::info!("clock: the world was replaced; following the new episode");
                let (mut old, cb, _) = sub;
                let _ = old.remove_on_tick(cb); // the old episode is gone; best effort
                match subscribe(clock) {
                    Some(s) => sub = s,
                    None => return true,
                }
                failures = 0;
            }
            Err(e) => {
                failures += 1;
                if failures >= CLOCK_MAX_FAILURES {
                    tracing::warn!("clock: CARLA stopped answering ({e}); reconnecting");
                    let (mut old, cb, _) = sub;
                    let _ = old.remove_on_tick(cb); // best effort on a dead connection
                    return true;
                }
            }
        }
    }
    let (mut old, cb, _) = sub;
    let _ = old.remove_on_tick(cb); // shutdown
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    const STEP: i64 = 50_000_000;

    /// CARLA's elapsed time: `k` steps of the f32 0.05 accumulated in f64, as the server does.
    fn carla_secs(base: f64, k: u32) -> f64 {
        (0..k).fold(base, |t, _| t + f64::from(0.05f32))
    }

    fn tick(episode: u64, secs: f64) -> Tick {
        Tick {
            episode,
            elapsed_ns: nanos(secs),
            delta_ns: nanos(f64::from(0.05f32)),
        }
    }

    /// Frames a wall-clock 50 ms apart, from `t0`.
    fn wall(t0: Instant, k: u32) -> Instant {
        t0 + Duration::from_millis(50 * u64::from(k))
    }

    /// The acceptance property: for any frame, the stamp computed from its CARLA time (a
    /// sensor's `timestamp()`, a snapshot's `elapsed_seconds`) is the `/clock` published for
    /// it, to the nanosecond -- also across an episode change.
    #[test]
    fn stamp_equals_clock_for_the_same_frame() {
        let mut c = EpisodeClock::new();
        let t0 = Instant::now();
        let base = 218_696.532_521_3;
        for k in 0..3000u32 {
            let secs = carla_secs(base, k);
            let out = c.on_frame(tick(1, secs), wall(t0, k));
            assert_eq!(out.publish_ns, Some(c.sim_ns(secs)), "frame {k}");
            assert_eq!(
                nanos_to_time(out.publish_ns.unwrap()),
                nanos_to_time(c.sim_ns(secs))
            );
        }
        for k in 0..100u32 {
            let secs = carla_secs(0.0, k + 1);
            let out = c.on_frame(tick(2, secs), wall(t0, 3000 + k));
            assert_eq!(
                out.publish_ns,
                Some(c.sim_ns(secs)),
                "new episode frame {k}"
            );
        }
    }

    /// Frames the subscription never saw only make the clock step further; it never goes
    /// back and never repeats.
    #[test]
    fn clock_is_monotonic_under_skipped_frames() {
        let mut c = EpisodeClock::new();
        let t0 = Instant::now();
        let mut prev = None;
        let mut k = 0u32;
        for (i, gap) in [1u32, 2, 1, 5, 1, 1, 17, 3, 1, 400, 1]
            .iter()
            .cycle()
            .take(500)
            .enumerate()
        {
            k += gap;
            let out = c.on_frame(tick(1, carla_secs(100.0, k)), wall(t0, i as u32));
            let v = out.publish_ns.expect("every frame published at 20 Hz");
            if let Some(p) = prev {
                assert!(v > p, "step {i}: {v} after {p}");
            }
            prev = Some(v);
        }
        // A duplicate frame (the same frame delivered twice) is not published again.
        let again = c.on_frame(tick(1, carla_secs(100.0, k)), wall(t0, 600));
        assert_eq!(again.publish_ns, None);
    }

    /// Asynchronous CARLA can tick every few milliseconds; /clock stays at or below 100 Hz,
    /// and the last frame before a pause still gets out.
    #[test]
    fn rate_is_capped_at_100_hz_and_the_last_frame_is_flushed() {
        let mut c = EpisodeClock::new();
        let t0 = Instant::now();
        let mut published = Vec::new();
        let n = 1000u32; // 2 s of frames at 500 Hz
        for k in 0..n {
            let at = t0 + Duration::from_millis(2 * u64::from(k));
            if let Some(v) = c
                .on_frame(tick(1, 100.0 + f64::from(k) * 0.002), at)
                .publish_ns
            {
                published.push((at, v));
            }
        }
        assert!(published.len() <= 201, "{} in 2 s", published.len());
        assert!(published.len() >= 190, "{} in 2 s", published.len());
        for w in published.windows(2) {
            assert!(w[1].0 - w[0].0 >= MIN_PUBLISH_INTERVAL);
        }
        // Too early to flush, then due.
        let last_at = t0 + Duration::from_millis(2 * u64::from(n - 1));
        let flushed = c
            .flush(last_at + Duration::from_millis(1))
            .or_else(|| c.flush(last_at + Duration::from_millis(20)));
        let last = c.sim_ns(100.0 + f64::from(n - 1) * 0.002);
        assert!(published.last().unwrap().1 == last || flushed == Some(last));
        assert_eq!(
            c.flush(last_at + Duration::from_secs(1)),
            None,
            "nothing left"
        );
    }

    /// Below 60 s of simulation time nothing is published, and that is reported once.
    #[test]
    fn uptime_gate_refuses_below_60_s_and_says_so_once() {
        let mut c = EpisodeClock::new();
        let t0 = Instant::now();
        let first = c.on_frame(tick(1, 12.5), t0);
        assert_eq!(first.publish_ns, None);
        assert_eq!(first.refused_ns, Some(12_500_000_000));
        let second = c.on_frame(tick(1, 59.95), wall(t0, 1));
        assert_eq!(second, FrameOutcome::default());
        let open = c.on_frame(tick(1, 60.0), wall(t0, 2));
        assert_eq!(open.publish_ns, Some(60_000_000_000));
    }

    /// A reload restarts `elapsed_seconds`; the new episode's time continues from the old
    /// one's last frame plus its step: `epoch += E_last + Δ_last`.
    #[test]
    fn episode_change_continues_one_step_after_the_last_frame() {
        let mut c = EpisodeClock::new();
        let t0 = Instant::now();
        let last = carla_secs(1000.0, 20);
        c.on_frame(tick(7, last), t0);
        let delta = nanos(f64::from(0.05f32));
        // New episode, first frame at elapsed 0.
        let out = c.on_frame(tick(8, 0.0), wall(t0, 1));
        let expected = nanos(last) + delta;
        assert_eq!(
            out.change,
            Some(EpochChange::Episode {
                old: 0,
                new: expected,
                sim_ns: expected
            })
        );
        assert_eq!(out.publish_ns, Some(expected));
        // And the next frame one step later.
        let next = c.on_frame(tick(8, f64::from(0.05f32)), wall(t0, 2));
        assert_eq!(next.publish_ns, Some(expected + delta));
        // Elapsed going backwards is a new episode even under the same id.
        let again = c.on_frame(tick(8, 0.0), wall(t0, 3));
        assert!(matches!(again.change, Some(EpochChange::Episode { .. })));
        assert!(again.publish_ns.unwrap() > expected + delta);
    }

    /// After a reconnect to a server whose episode is new (CARLA restarted), continue from
    /// the last /clock published plus one step; a reconnect to the same episode changes
    /// nothing.
    #[test]
    fn reconnect_continues_from_the_last_published_clock() {
        let mut c = EpisodeClock::new();
        let t0 = Instant::now();
        let last = carla_secs(5000.0, 10);
        let published = c.on_frame(tick(1, last), t0).publish_ns.unwrap();

        // Same server, same episode: nothing changes.
        c.mark_reconnect();
        let same = c.on_frame(tick(1, carla_secs(5000.0, 11)), wall(t0, 1));
        assert_eq!(same.change, None);
        let published = same.publish_ns.unwrap().max(published);

        // Restarted server: a new episode at a small uptime.
        c.mark_reconnect();
        let out = c.on_frame(tick(99, 3.25), wall(t0, 2));
        let delta = nanos(f64::from(0.05f32));
        match out.change {
            Some(EpochChange::Reconnect { sim_ns, .. }) => assert_eq!(sim_ns, published + delta),
            other => panic!("expected a reconnect epoch, got {other:?}"),
        }
        assert_eq!(out.publish_ns, Some(published + delta));
        let next = c.on_frame(tick(99, 3.25 + f64::from(0.05f32)), wall(t0, 3));
        assert_eq!(next.publish_ns, Some(published + 2 * delta));
    }

    /// The shared table: carla-scenario-bridge's `episode_clock.rs` carries the same cases
    /// and must give the same nanoseconds. Exact halves round away from zero (not to even);
    /// CARLA's f32 step 0.050000000745 is accumulated in f64, as the server does.
    const NANOS_CASES: &[(f64, i64)] = &[
        (0.0, 0),
        (0.05000000074505806, 50_000_001),
        (5e-10, 1),
        (1.5e-9, 2),
        (2.5e-9, 3),
        (-2.5e-9, -3),
        (3.5e-9, 4),
        (0.10000000149011612, 100_000_001),
        (0.15000000223517418, 150_000_002),
        (67.0000009983778, 67_000_000_998),
        (171.60000255703926, 171_600_002_557),
        (1000.0000149011612, 1_000_000_014_901),
        (237000.05000000075, 237_000_050_000_001),
        (237171.60000255704, 237_171_600_002_557),
        (1790611998.05, 1_790_611_998_049_999_872),
    ];

    /// `(base seconds, steps of f32 0.05, ns)`: the accumulations behind the table rows.
    const STEP_CASES: &[(f64, u32, i64)] = &[
        (0.0, 1, 50_000_001),
        (0.0, 3, 150_000_002),
        (0.0, 1340, 67_000_000_998),
        (0.0, 3432, 171_600_002_557),
        (0.0, 20000, 1_000_000_014_901),
        (237000.0, 1, 237_000_050_000_001),
        (237000.0, 3432, 237_171_600_002_557),
    ];

    #[test]
    fn nanos_matches_the_shared_table() {
        for &(secs, ns) in NANOS_CASES {
            assert_eq!(nanos(secs), ns, "nanos({secs:?})");
        }
        for &(base, k, ns) in STEP_CASES {
            assert_eq!(nanos(carla_secs(base, k)), ns, "{k} f32 steps from {base}");
        }
    }

    /// The epoch is the sum of the rounded terms, the same as csb's: after an episode change
    /// the sim time is `nanos(elapsed) + nanos(E_last) + nanos(Δ_last)`.
    #[test]
    fn epoch_sums_rounded_terms_like_csb() {
        let mut c = EpisodeClock::new();
        let t0 = Instant::now();
        let e_last = carla_secs(0.0, 3432);
        c.on_frame(tick(1, e_last), t0);
        let elapsed = carla_secs(0.0, 1340);
        c.on_frame(tick(2, elapsed), t0 + Duration::from_secs(1));
        assert_eq!(
            c.sim_ns(elapsed),
            67_000_000_998 + 171_600_002_557 + 50_000_001
        );
    }

    #[test]
    fn nanos_split_into_sec_and_nanosec() {
        let t = nanos_to_time(1_500_000_000);
        assert_eq!((t.sec, t.nanosec), (1, 500_000_000));
        let t = nanos_to_time(-1);
        assert_eq!((t.sec, t.nanosec), (0, 0));
    }

    #[test]
    fn step_constant_matches_carla_f32_step_to_the_nanosecond() {
        assert_eq!(nanos(f64::from(0.05f32)), STEP + 1);
    }
}
