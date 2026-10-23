//! Simulation time, taken from CARLA alone (carla-scenario-bridge roadmap 051, decision 5).
//!
//! ```text
//! sim_ns = nanos(elapsed_seconds) + BASE_NS
//! ```
//!
//! `elapsed_seconds` is the frame time CARLA puts in every `WorldSnapshot` and every
//! `SensorData`. Every stamp this bridge publishes and every `/clock` message goes through
//! [`SimClock::stamp`], so a sensor sample, the odometry of its frame and that frame's
//! `/clock` carry the same nanosecond value -- and the scenario bridge, computing the same
//! from the same frame, reports it to SSv2: nothing is shared, stored or sent between them.
//! `BASE_NS` (1000 s, the world protocol's `sim_time::BASE_NS`) keeps ROS time away from 0
//! (ROS's "no clock yet") and from the small values for which `now() - duration` goes
//! negative in tf2 and Autoware's monitors.
//!
//! # An episode is this process's lifetime
//!
//! Time is monotonic within a CARLA episode (one world, from its load to the next load or the
//! server's death) and only there: CARLA restarts `elapsed_seconds` at 0 for every episode.
//! Autoware has no answer to a clock that goes back, so this bridge attaches to the episode
//! it first sees and **exits** -- with [`EPISODE_CHANGED_EXIT`] and a message naming both
//! episodes -- the moment it sees another: a world load, or a reconnect that finds a
//! restarted CARLA. It exits from the frame that shows the change, before anything stamped
//! in the new episode is published. Its launch declares it required, so the exit ends the
//! whole Autoware stack, and the stack comes back on the new episode (and the new town's
//! map) through whatever started it (docs/design/time-and-ticking.md, "Episodes and vehicle
//! stacks").
//!
//! `/clock` comes from an `on_tick` subscription of its own, on a client of its own
//! ([`spawn_clock_source`]), established when CARLA is reachable -- not when a hero is found
//! -- so it runs between scenarios (wall-paced, CARLA free-runs asynchronously) and during
//! them (one step per frame the scenario ticks).

use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    time::{Duration, Instant},
};

use crate::error::Result;

/// 1000 s, in ns: added to every simulation time. Must stay identical to
/// swb_world_protocol's `sim_time::BASE_NS` (scenario-world-bridge's
/// `scripts/test_sim_time.py` checks).
pub const BASE_NS: i64 = 1_000_000_000_000;

/// The exit status when this bridge's episode is gone (roadmap 051): not a failure of the
/// bridge, the end of the stack's episode.
pub const EPISODE_CHANGED_EXIT: i32 = 3;

/// Seconds to integer nanoseconds: `round(secs * 1e9)`, half away from zero (`f64::round`).
/// The one conversion every stamp and `/clock` value goes through, which is what makes them
/// bit-exact with one another. swb_world_protocol's `sim_time::nanos` must stay identical
/// (it reports the same integer to SSv2); both carry the same table of cases.
pub fn nanos(secs: f64) -> i64 {
    (secs * 1e9).round() as i64
}

/// The simulation time of a frame `elapsed` seconds into its episode, ns.
pub fn sim_ns(elapsed_secs: f64) -> i64 {
    nanos(elapsed_secs) + BASE_NS
}

/// Split nanoseconds into a ROS `Time`. ROS time is unsigned; negative clamps to zero.
pub fn nanos_to_time(nanos: i64) -> builtin_interfaces::msg::Time {
    let nanos = nanos.max(0);
    builtin_interfaces::msg::Time {
        sec: (nanos / 1_000_000_000) as i32,
        nanosec: (nanos % 1_000_000_000) as u32,
    }
}

/// At most one `/clock` per this interval: 100 Hz. CARLA running asynchronously with
/// nothing to render can tick far faster than anything needs a clock.
pub const MIN_PUBLISH_INTERVAL: Duration = Duration::from_millis(10);

/// One CARLA frame, as the clock needs it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tick {
    /// Episode id (`WorldSnapshot::id`, which libcarla takes from the episode).
    pub episode: u64,
    pub elapsed_ns: i64,
}

impl Tick {
    pub fn from_snapshot(snapshot: &carla::client::WorldSnapshot) -> Self {
        Self {
            episode: snapshot.id(),
            elapsed_ns: nanos(snapshot.timestamp().elapsed_seconds),
        }
    }
}

/// What one frame does to the clock.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct FrameOutcome {
    /// Publish this `/clock` value now (ns).
    pub publish_ns: Option<i64>,
    /// The frame is of another episode than the one this bridge attached to: `(attached,
    /// seen)`. The process must end before it publishes anything more.
    pub episode_changed: Option<(u64, u64)>,
}

/// The episode this bridge attached to, and the rate cap's state.
#[derive(Debug, Default)]
pub struct EpisodeClock {
    /// The first episode seen; every frame after must be of it.
    episode: Option<u64>,
    /// The last value published, and when.
    published: Option<(i64, Instant)>,
    /// A value the rate cap held back, to publish once due.
    held: Option<i64>,
}

impl EpisodeClock {
    pub fn new() -> Self {
        Self::default()
    }

    /// Whether `episode` is this bridge's (attaching to it if none is yet): `Err((attached,
    /// seen))` when it is another.
    pub fn check_episode(&mut self, episode: u64) -> std::result::Result<(), (u64, u64)> {
        match self.episode {
            None => {
                self.episode = Some(episode);
                Ok(())
            }
            Some(e) if e == episode => Ok(()),
            Some(e) => Err((e, episode)),
        }
    }

    pub fn on_frame(&mut self, tick: Tick, now: Instant) -> FrameOutcome {
        if let Err(change) = self.check_episode(tick.episode) {
            return FrameOutcome {
                publish_ns: None,
                episode_changed: Some(change),
            };
        }
        let sim = tick.elapsed_ns + BASE_NS;
        match self.published {
            // The same frame delivered twice, or one older than what went out: nothing new.
            Some((last, _)) if sim <= last => FrameOutcome::default(),
            Some((_, at)) if now.duration_since(at) < MIN_PUBLISH_INTERVAL => {
                self.held = Some(sim);
                FrameOutcome::default()
            }
            _ => {
                self.published = Some((sim, now));
                self.held = None;
                FrameOutcome {
                    publish_ns: Some(sim),
                    episode_changed: None,
                }
            }
        }
    }

    /// The value the rate cap held back, once it is due.
    pub fn flush(&mut self, now: Instant) -> Option<i64> {
        let held = self.held?;
        match self.published {
            Some((_, at)) if now.duration_since(at) < MIN_PUBLISH_INTERVAL => None,
            _ => {
                self.published = Some((held, now));
                self.held = None;
                Some(held)
            }
        }
    }
}

/// What the process runs before an [`EPISODE_CHANGED_EXIT`] (the binary's log flush: an exit
/// runs no destructors).
static BEFORE_EXIT: std::sync::OnceLock<fn()> = std::sync::OnceLock::new();

/// Register what to run before the episode-change exit (once).
pub fn before_exit(f: fn()) {
    let _ = BEFORE_EXIT.set(f);
}

/// End the process: this bridge's episode is gone (see the module docs).
fn end_of_episode(attached: u64, seen: u64) -> ! {
    tracing::error!(
        "CARLA's episode changed ({attached} -> {seen}): a world load, or a restarted CARLA. \
         Simulation time starts again in a new episode and Autoware cannot follow it back, so \
         this bridge exits (status {EPISODE_CHANGED_EXIT}) before publishing anything of the \
         new one; its launch ends the Autoware stack with it, and the stack comes back on the \
         new episode (roadmap 051)"
    );
    if let Some(f) = BEFORE_EXIT.get() {
        f();
    }
    std::process::exit(EPISODE_CHANGED_EXIT);
}

/// The shared clock: the episode every stamp belongs to, and the `/clock` publisher. Cheap
/// to clone.
#[derive(Clone)]
pub struct SimClock {
    state: Arc<Mutex<EpisodeClock>>,
    publisher: Arc<rclrs::Publisher<rclrs::vendor::rosgraph_msgs::msg::Clock>>,
    /// `publish_clock`: an emergency off. Stamps still follow the frames when it is false.
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
        nanos_to_time(sim_ns(elapsed_secs))
    }

    /// A world this bridge is about to use (the hero session's, each frame): exits if it is
    /// of another episode than this bridge's.
    pub fn check_episode(&self, episode: u64) {
        let result = self.lock().check_episode(episode);
        if let Err((attached, seen)) = result {
            end_of_episode(attached, seen);
        }
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
        if let Some((attached, seen)) = out.episode_changed {
            end_of_episode(attached, seen);
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
/// The clock client's RPC timeout.
const CLOCK_CLIENT_TIMEOUT: Duration = Duration::from_secs(10);

/// Run `/clock` on a CARLA client of its own, from connect, for the life of the node.
///
/// Its own client, because the session client is dropped and rebuilt after every despawn
/// to shed the sensor streams the scenario runner destroyed (`SessionExit::VehicleLost`),
/// and the clock must not stop for that. It never ticks the world. A reconnect that finds
/// another episode ends the process at its first frame ([`SimClock::on_frame`]).
pub fn spawn_clock_source(
    address: String,
    port: u16,
    clock: SimClock,
    running: Arc<AtomicBool>,
) -> std::thread::JoinHandle<()> {
    std::thread::Builder::new()
        .name("acb-clock".into())
        .spawn(move || {
            while running.load(Ordering::SeqCst) {
                let mut client = match carla::client::Client::connect(&address, port, None) {
                    Ok(mut c) => {
                        if let Err(e) = c.set_timeout(CLOCK_CLIENT_TIMEOUT) {
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
                follow_frames(&mut client, &clock, &running);
            }
        })
        .expect("spawn the clock thread")
}

/// Subscribe and keep the subscription on the current world. Returns when the connection
/// looks dead (or on shutdown). A replaced world is another episode: its first frame ends
/// the process.
fn follow_frames(client: &mut carla::client::Client, clock: &SimClock, running: &AtomicBool) {
    let subscribe = |client: &carla::client::Client,
                     clock: &SimClock|
     -> Option<(carla::client::World, carla::client::OnTickId, u64)> {
        let mut world = client.world().ok()?;
        let id = world.id().ok()?;
        let sink = clock.clone();
        let cb = world
            .on_tick(move |snapshot| sink.on_frame(Tick::from_snapshot(&snapshot)))
            .ok()?;
        Some((world, cb, id))
    };
    let Some(mut sub) = subscribe(client, clock) else {
        std::thread::sleep(Duration::from_secs(2));
        return;
    };
    // A reconnect onto another episode ends here, before any of its frames.
    clock.check_episode(sub.2);
    tracing::info!("/clock follows CARLA frames (episode {})", sub.2);
    let mut failures = 0u32;
    let mut last_check = Instant::now();
    let mut probe = crate::conn_probe::ConnectionProbe::new(CLOCK_CLIENT_TIMEOUT);
    while running.load(Ordering::SeqCst) {
        std::thread::sleep(CLOCK_POLL);
        // Frames arriving prove nothing: after a server restart they come from the new
        // server over a subscription whose RPC connection is dead (crate::conn_probe).
        if probe.dead(client) {
            tracing::warn!("clock: the CARLA connection is dead; reconnecting");
            let (mut old, cb, _) = sub;
            let _ = old.remove_on_tick(cb); // local; the connection is dead
            return;
        }
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
            Ok((_, id)) => {
                // Replaced: another episode, which ends this bridge.
                clock.check_episode(id);
                let (mut old, cb, _) = sub;
                let _ = old.remove_on_tick(cb);
                match subscribe(client, clock) {
                    Some(s) => sub = s,
                    None => return,
                }
                failures = 0;
            }
            Err(e) => {
                failures += 1;
                if failures >= CLOCK_MAX_FAILURES {
                    tracing::warn!("clock: CARLA stopped answering ({e}); reconnecting");
                    let (mut old, cb, _) = sub;
                    let _ = old.remove_on_tick(cb); // best effort on a dead connection
                    return;
                }
            }
        }
    }
    let (mut old, cb, _) = sub;
    let _ = old.remove_on_tick(cb); // shutdown
}

#[cfg(test)]
mod tests {
    use super::*;

    /// CARLA's elapsed time: `k` steps of the f32 0.05 accumulated in f64, as the server does.
    fn carla_secs(base: f64, k: u32) -> f64 {
        (0..k).fold(base, |t, _| t + f64::from(0.05f32))
    }

    fn tick(episode: u64, secs: f64) -> Tick {
        Tick {
            episode,
            elapsed_ns: nanos(secs),
        }
    }

    /// Frames a wall-clock 50 ms apart, from `t0`.
    fn wall(t0: Instant, k: u32) -> Instant {
        t0 + Duration::from_millis(50 * u64::from(k))
    }

    /// `(seconds, ns)`: must match swb_world_protocol's `sim_time` table, case for case.
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

    #[test]
    fn nanos_matches_the_shared_table() {
        for &(secs, ns) in NANOS_CASES {
            assert_eq!(nanos(secs), ns, "nanos({secs:?})");
        }
        assert_eq!(sim_ns(0.0), 1_000_000_000_000);
    }

    /// The acceptance property: for any frame, the stamp computed from its CARLA time (a
    /// sensor's `timestamp()`, a snapshot's `elapsed_seconds`) is the `/clock` published for
    /// it, to the nanosecond.
    #[test]
    fn stamp_equals_clock_for_the_same_frame() {
        let mut c = EpisodeClock::new();
        let t0 = Instant::now();
        for k in 0..3000u32 {
            let secs = carla_secs(0.0, k);
            let out = c.on_frame(tick(1, secs), wall(t0, k));
            assert_eq!(out.publish_ns, Some(sim_ns(secs)), "frame {k}");
        }
    }

    /// The first frame's episode is this bridge's; a frame of another ends it, whatever its
    /// time, and nothing of it is published.
    #[test]
    fn another_episode_ends_the_bridge_and_publishes_nothing() {
        let mut c = EpisodeClock::new();
        let t0 = Instant::now();
        assert!(c.on_frame(tick(7, 300.0), t0).publish_ns.is_some());
        let out = c.on_frame(tick(9, 0.05), wall(t0, 1));
        assert_eq!(out.publish_ns, None);
        assert_eq!(out.episode_changed, Some((7, 9)));
        assert_eq!(c.check_episode(7), Ok(()));
        assert_eq!(c.check_episode(8), Err((7, 8)));
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
        let last_at = t0 + Duration::from_millis(2 * u64::from(n - 1));
        let flushed = c
            .flush(last_at + Duration::from_millis(1))
            .or_else(|| c.flush(last_at + Duration::from_millis(20)));
        let last = sim_ns(100.0 + f64::from(n - 1) * 0.002);
        assert!(published.last().unwrap().1 == last || flushed == Some(last));
        assert_eq!(
            c.flush(last_at + Duration::from_secs(1)),
            None,
            "nothing left"
        );
    }

    #[test]
    fn negative_time_clamps_to_zero() {
        let t = nanos_to_time(-5);
        assert_eq!((t.sec, t.nanosec), (0, 0));
    }
}
