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
//! A reconnect to a restarted CARLA applies the same rule to the last frame this bridge
//! received from the old server, anchored, like any episode, at the new episode's
//! `elapsed = 0`: the new server's uptime before we reached it shows as time that passed,
//! never as time going back. The scenario bridge does the same with the last frame it
//! ticked (csb roadmap 016), so the two still agree unless the crash swallowed that
//! frame's callback here -- then they differ by one step. Anchoring at the first frame we
//! happen to see instead would make the epoch depend on when each bridge reconnected.
//!
//! # Joining mid-session: the epoch marker
//!
//! The rule needs the episode changes to have been watched. A bridge started after a reload
//! (a vehicle side restarted mid-session) would start at epoch 0 while csb and every bridge
//! that saw the reload carry the earlier episodes' length -- and publish a `/clock` that far
//! behind its peers for the rest of the session. So whoever reloads the world (csb) records
//! the new epoch *in the new world*: an actor of [`EPOCH_MARKER_BLUEPRINT`] (no mesh, no
//! collision, no physics) whose `role_name` is `epoch_ns:<integer>`. `load_world` destroys
//! it with everything else, so every episode carries its own.
//!
//! A bridge that has not watched the episode begin -- its first frame ever, or the first
//! after a connection on which no frame arrived -- withholds `/clock` until it has read the
//! marker from that episode, then adopts it ([`EpisodeClock::on_marker`]); no marker means
//! epoch 0, which is the behaviour without csb. A bridge that watched the change keeps its
//! own epoch and only compares: a marker that disagrees is logged, never adopted. Still
//! CARLA and nothing else: no csb, no file, no channel between the bridges.

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

/// The blueprint of the epoch marker: an empty actor, with no mesh, no collision and no
/// physics, that still carries a settable `role_name` (CARLA 0.9.16). Must stay identical to
/// carla-scenario-bridge's `epoch_marker::BLUEPRINT`.
pub const EPOCH_MARKER_BLUEPRINT: &str = "util.actor.empty";

/// `role_name` of the epoch marker: this prefix, then the episode's epoch in integer
/// nanoseconds (`epoch_ns:150000002`). Must stay identical to csb's `epoch_marker::PREFIX`.
pub const EPOCH_MARKER_PREFIX: &str = "epoch_ns:";

/// Attempts to read an episode's marker: a failed read, or (when only comparing) a marker
/// csb has not written yet, is retried this many times in all.
pub const MARKER_ATTEMPTS: u32 = 3;

/// Between two attempts to read an episode's marker, and only after a new frame.
pub const MARKER_RETRY: Duration = Duration::from_secs(1);

/// The epoch an epoch marker's `role_name` records, if it is one.
pub fn parse_epoch_marker(role_name: &str) -> Option<i64> {
    let digits = role_name.strip_prefix(EPOCH_MARKER_PREFIX)?;
    if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    digits.parse().ok()
}

/// The epoch the world records, from `(actor id, role_name)` of its marker-blueprint actors:
/// the newest valid marker (highest actor id; CARLA numbers actors upwards within an
/// episode). `None` if there is none.
pub fn epoch_from_markers<S: AsRef<str>>(
    markers: impl IntoIterator<Item = (u32, S)>,
) -> Option<i64> {
    markers
        .into_iter()
        .filter_map(|(id, role)| parse_epoch_marker(role.as_ref()).map(|epoch| (id, epoch)))
        .max_by_key(|&(id, _)| id)
        .map(|(_, epoch)| epoch)
}

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

/// What reading an episode's epoch marker found, for the log.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MarkerEvent {
    /// Joined mid-session: the epoch is the marker's. `sim_ns` is the current frame's time.
    Adopted { epoch_ns: i64, sim_ns: i64 },
    /// Joined, and the episode carries no marker: epoch 0, as without csb.
    NoMarker { sim_ns: i64 },
    /// Joined, and the marker could not be read in [`MARKER_ATTEMPTS`] tries: epoch 0.
    GaveUp { sim_ns: i64 },
    /// Watched the change, and the marker records the same epoch.
    Agrees { epoch_ns: i64 },
    /// Watched the change, and the marker records another epoch. Ours is kept.
    Disagrees { own_ns: i64, marker_ns: i64 },
}

/// Why the current episode's marker is read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MarkerPurpose {
    /// The epoch is unknown until it is read; `/clock` is withheld meanwhile.
    Adopt,
    /// The epoch is ours; the marker is only compared with it.
    Verify,
}

#[derive(Debug, Clone, Copy)]
struct MarkerCheck {
    episode: u64,
    purpose: MarkerPurpose,
    attempts: u32,
    last_attempt: Option<Instant>,
    /// A frame of this episode arrived since the last attempt (the client's actor list is
    /// only as new as its last frame).
    fresh_frame: bool,
}

impl MarkerCheck {
    fn new(episode: u64, purpose: MarkerPurpose) -> Self {
        Self {
            episode,
            purpose,
            attempts: 0,
            last_attempt: None,
            fresh_frame: true,
        }
    }
}

/// The clock's state: pure, so the rules are testable without CARLA or ROS.
#[derive(Debug, Default)]
pub struct EpisodeClock {
    epoch_ns: i64,
    /// The epoch is known: this bridge watched the episode begin or read its marker. Until
    /// then nothing is published.
    epoch_known: bool,
    /// The current episode's marker still has to be read.
    marker_check: Option<MarkerCheck>,
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
        if !self.epoch_known {
            // Nothing watched yet: the epoch is the marker's of whichever episode this is.
            // A change before the marker was read only retargets the read.
            match &mut self.marker_check {
                Some(check) if check.episode == tick.episode => check.fresh_frame = true,
                _ => self.marker_check = Some(MarkerCheck::new(tick.episode, MarkerPurpose::Adopt)),
            }
            self.last = Some(tick);
            return None;
        }
        if let Some(check) = self
            .marker_check
            .as_mut()
            .filter(|c| c.episode == tick.episode)
        {
            check.fresh_frame = true;
        }
        let change = match self.last {
            Some(last) if tick.episode != last.episode || tick.elapsed_ns < last.elapsed_ns => {
                let old = self.epoch_ns;
                // The same rule either way; a reconnect is only logged differently, because
                // the old server's last frame may not have reached us.
                self.epoch_ns += last.elapsed_ns + last.delta_ns;
                self.marker_check = Some(MarkerCheck::new(tick.episode, MarkerPurpose::Verify));
                if reconnected {
                    Some(EpochChange::Reconnect {
                        old,
                        new: self.epoch_ns,
                        sim_ns: tick.elapsed_ns + self.epoch_ns,
                    })
                } else {
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
        if !self.epoch_known {
            self.pending_ns = None; // withheld until the marker is read
            return FrameOutcome::default();
        }
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

    /// The episode whose marker should be read now, if any: a frame of it has arrived since
    /// the last attempt, and the last attempt is [`MARKER_RETRY`] old.
    pub fn marker_lookup_due(&self, now: Instant) -> Option<u64> {
        let check = self.marker_check?;
        let current = self.last.is_some_and(|t| t.episode == check.episode);
        let spaced = check
            .last_attempt
            .is_none_or(|at| now.saturating_duration_since(at) >= MARKER_RETRY);
        (current && check.fresh_frame && spaced).then_some(check.episode)
    }

    /// What reading `episode`'s marker found: `Ok(Some(epoch))`, `Ok(None)` for no marker,
    /// `Err(())` for a failed read. A result for an episode that is no longer current is
    /// dropped.
    pub fn on_marker(
        &mut self,
        episode: u64,
        read: std::result::Result<Option<i64>, ()>,
        now: Instant,
    ) -> Option<MarkerEvent> {
        let mut check = self.marker_check.filter(|c| c.episode == episode)?;
        let last = self.last.filter(|t| t.episode == episode)?;
        check.attempts += 1;
        check.last_attempt = Some(now);
        check.fresh_frame = false;
        let exhausted = check.attempts >= MARKER_ATTEMPTS;
        self.marker_check = Some(check);
        match (check.purpose, read) {
            (MarkerPurpose::Adopt, Err(())) if !exhausted => None,
            (MarkerPurpose::Adopt, read) => {
                self.marker_check = None;
                self.epoch_known = true;
                self.epoch_ns = read.ok().flatten().unwrap_or(0);
                let sim_ns = last.elapsed_ns + self.epoch_ns;
                // The withheld frame goes out through `flush`, under the usual gate and cap.
                if sim_ns >= MIN_SIM_TIME_NS && self.last_published_ns.is_none_or(|p| sim_ns > p) {
                    self.pending_ns = Some(sim_ns);
                }
                Some(match read {
                    Ok(Some(epoch_ns)) => MarkerEvent::Adopted { epoch_ns, sim_ns },
                    Ok(None) => MarkerEvent::NoMarker { sim_ns },
                    Err(()) => MarkerEvent::GaveUp { sim_ns },
                })
            }
            (MarkerPurpose::Verify, Ok(Some(marker_ns))) => {
                self.marker_check = None;
                Some(if marker_ns == self.epoch_ns {
                    MarkerEvent::Agrees {
                        epoch_ns: marker_ns,
                    }
                } else {
                    MarkerEvent::Disagrees {
                        own_ns: self.epoch_ns,
                        marker_ns,
                    }
                })
            }
            // Not written yet (csb writes it just after the load), or not readable: retry,
            // then stop comparing. Our epoch stands either way.
            (MarkerPurpose::Verify, _) => {
                if exhausted {
                    self.marker_check = None;
                }
                None
            }
        }
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
                 reconnect (CARLA restarted?): the episode rule was applied to the last frame \
                 received from the old server. carla_scenario_bridge does the same with the \
                 last frame it ticked; the epochs differ by one step if that frame's callback \
                 never arrived here",
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

    /// Read the current episode's epoch marker if one is due (see [`EpisodeClock::on_marker`]),
    /// and publish the frame that was withheld for it. Called from the clock thread, never
    /// from CARLA's callback thread: it is an RPC.
    fn check_marker(&self, client: &carla::client::Client) {
        let Some(episode) = self.lock().marker_lookup_due(Instant::now()) else {
            return;
        };
        let read = read_epoch_marker(client, episode).map_err(|e| {
            tracing::debug!("clock: could not read the epoch marker of episode {episode}: {e}");
        });
        let event = self.lock().on_marker(episode, read, Instant::now());
        let secs = |ns: i64| ns as f64 / 1e9;
        match event {
            Some(MarkerEvent::Adopted { epoch_ns, sim_ns }) => tracing::info!(
                "Joined episode {episode} mid-session: epoch {:.9} from CARLA's epoch marker \
                 (written by whoever reloaded the world), sim_time continues at {:.9}",
                secs(epoch_ns),
                secs(sim_ns)
            ),
            Some(MarkerEvent::NoMarker { sim_ns }) => tracing::info!(
                "Episode {episode} carries no epoch marker (nothing reloaded it, or no \
                 carla_scenario_bridge): epoch 0, sim_time {:.9}",
                secs(sim_ns)
            ),
            Some(MarkerEvent::GaveUp { sim_ns }) => tracing::warn!(
                "Could not read episode {episode}'s epoch marker in {MARKER_ATTEMPTS} attempts; \
                 starting from epoch 0 (sim_time {:.9}). If the world was reloaded before this \
                 bridge started, /clock trails the other bridges' by the earlier episodes",
                secs(sim_ns)
            ),
            Some(MarkerEvent::Agrees { epoch_ns }) => tracing::debug!(
                "Episode {episode}'s epoch marker agrees: {:.9}",
                secs(epoch_ns)
            ),
            Some(MarkerEvent::Disagrees { own_ns, marker_ns }) => tracing::warn!(
                "Episode {episode}'s epoch marker records {:.9}, this bridge's epoch is {:.9} \
                 ({:+.9} s); keeping ours, which was applied to the frames this bridge saw. \
                 After a CARLA restart a one-step difference is expected (the old server's \
                 last frame); anything else means the bridges' clocks differ",
                secs(marker_ns),
                secs(own_ns),
                secs(own_ns - marker_ns)
            ),
            None => {}
        }
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

/// The epoch recorded in `episode`'s world: its newest epoch marker, or `None`. `Err` if the
/// actors cannot be listed, or the client is already on another episode.
fn read_epoch_marker(
    client: &carla::client::Client,
    episode: u64,
) -> std::result::Result<Option<i64>, String> {
    use carla::client::ActorBase;
    let world = client.world().map_err(|e| e.to_string())?;
    let current = world.id().map_err(|e| e.to_string())?;
    if current != episode {
        return Err(format!("the client is on episode {current} now"));
    }
    // The client's actor list is as new as its last frame; the caller only asks after one.
    let markers = world
        .actors()
        .and_then(|actors| actors.filter(EPOCH_MARKER_BLUEPRINT))
        .map_err(|e| e.to_string())?;
    Ok(epoch_from_markers(markers.iter().filter_map(|actor| {
        let role = actor
            .attributes()
            .ok()?
            .iter()
            .find(|a| a.id() == "role_name")
            .map(|a| a.value_string())?;
        Some((actor.id(), role))
    })))
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
        clock.check_marker(client);
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

    /// A clock that watched its episode begin (epoch 0, known), as a bridge started before
    /// anything reloaded the world is once it has read the marker-less first episode.
    fn watching() -> EpisodeClock {
        EpisodeClock {
            epoch_known: true,
            ..Default::default()
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
        let mut c = watching();
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
        let mut c = watching();
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
        let mut c = watching();
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
        let mut c = watching();
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
        let mut c = watching();
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

    /// After a reconnect to a server whose episode is new (CARLA restarted), apply the
    /// episode rule to the last frame received: the new episode's elapsed 0 lands one step
    /// after it, whatever uptime the new server had when we reached it. A reconnect to the
    /// same episode changes nothing.
    #[test]
    fn reconnect_continues_from_the_last_published_clock() {
        let mut c = watching();
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
            Some(EpochChange::Reconnect { sim_ns, .. }) => {
                assert_eq!(sim_ns, published + delta + nanos(3.25))
            }
            other => panic!("expected a reconnect epoch, got {other:?}"),
        }
        assert_eq!(out.publish_ns, Some(published + delta + nanos(3.25)));
        let next = c.on_frame(tick(99, 3.25 + f64::from(0.05f32)), wall(t0, 3));
        assert_eq!(
            next.publish_ns,
            Some(published + delta + nanos(3.25 + f64::from(0.05f32)))
        );
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
        let mut c = watching();
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

    /// The observed case (2026-10-13): a 0.15 s episode, then Town05. A bridge that watched
    /// the reload carries epoch 0.150000002; one started after it reads the marker csb wrote
    /// into the new world and publishes the same `/clock`, to the nanosecond, from its first
    /// value on.
    #[test]
    fn late_join_adopts_the_marker_and_matches_a_watching_bridge() {
        let t0 = Instant::now();
        let mut watched = watching();
        watched.on_frame(tick(1, carla_secs(0.0, 2)), t0);
        let e_last = carla_secs(0.0, 2);
        let epoch = nanos(e_last) + nanos(f64::from(0.05f32));
        assert_eq!(epoch, 150_000_002);

        let mut late = EpisodeClock::new();
        let secs = 4000.0;
        let first = late.on_frame(tick(2, secs), t0);
        assert_eq!(
            first,
            FrameOutcome::default(),
            "withheld until the marker is read"
        );
        assert_eq!(late.marker_lookup_due(t0), Some(2));
        let event = late.on_marker(2, Ok(Some(epoch)), t0);
        let sim = nanos(secs) + epoch;
        assert_eq!(
            event,
            Some(MarkerEvent::Adopted {
                epoch_ns: epoch,
                sim_ns: sim
            })
        );
        assert_eq!(late.marker_lookup_due(t0), None, "read once");
        // The withheld frame goes out at once.
        assert_eq!(late.flush(t0 + MIN_PUBLISH_INTERVAL), Some(sim));

        // From here on the two bridges publish the same values for the same frames.
        let w = watched.on_frame(tick(2, 0.0), wall(t0, 1));
        assert!(matches!(w.change, Some(EpochChange::Episode { new, .. }) if new == epoch));
        for k in 1..50u32 {
            let secs = carla_secs(secs, k);
            let a = watched.on_frame(tick(2, secs), wall(t0, 1 + k)).publish_ns;
            let b = late.on_frame(tick(2, secs), wall(t0, 1 + k)).publish_ns;
            assert_eq!(a, b, "frame {k}");
            assert_eq!(late.sim_ns(secs), watched.sim_ns(secs));
        }
        // And both apply the rule to the next reload alike.
        let a = watched.on_frame(tick(3, 0.0), wall(t0, 60));
        let b = late.on_frame(tick(3, 0.0), wall(t0, 60));
        assert_eq!(a.change, b.change);
        assert_eq!(a.publish_ns, b.publish_ns);
    }

    /// No csb, nothing reloaded: no marker, epoch 0 -- the behaviour before markers.
    #[test]
    fn no_marker_means_epoch_zero() {
        let t0 = Instant::now();
        let mut c = EpisodeClock::new();
        assert_eq!(c.on_frame(tick(5, 300.0), t0), FrameOutcome::default());
        assert_eq!(
            c.on_marker(5, Ok(None), t0),
            Some(MarkerEvent::NoMarker {
                sim_ns: nanos(300.0)
            })
        );
        assert_eq!(c.flush(t0), Some(nanos(300.0)));
        let next = c.on_frame(tick(5, 300.05), wall(t0, 1));
        assert_eq!(next.publish_ns, Some(nanos(300.05)));
    }

    /// Nothing is due before a frame, and a failed read is retried -- a second later, after
    /// a new frame -- then given up on with epoch 0 rather than withholding `/clock` forever.
    #[test]
    fn failed_reads_are_retried_then_epoch_zero() {
        let t0 = Instant::now();
        let mut c = EpisodeClock::new();
        assert_eq!(
            c.marker_lookup_due(t0),
            None,
            "no frame, no episode to read"
        );
        c.on_frame(tick(5, 300.0), t0);
        for attempt in 1..MARKER_ATTEMPTS {
            let at = t0 + MARKER_RETRY * (attempt - 1);
            assert_eq!(c.marker_lookup_due(at), Some(5), "attempt {attempt}");
            assert_eq!(c.on_marker(5, Err(()), at), None);
            assert_eq!(
                c.marker_lookup_due(at + MARKER_RETRY),
                None,
                "needs a new frame"
            );
            let out = c.on_frame(tick(5, 300.0 + f64::from(attempt)), at + MARKER_RETRY / 2);
            assert_eq!(out, FrameOutcome::default(), "still withheld");
            assert_eq!(c.marker_lookup_due(at + MARKER_RETRY / 2), None, "too soon");
        }
        let at = t0 + MARKER_RETRY * (MARKER_ATTEMPTS - 1);
        assert_eq!(c.marker_lookup_due(at), Some(5));
        let last = nanos(300.0 + f64::from(MARKER_ATTEMPTS - 1));
        assert_eq!(
            c.on_marker(5, Err(()), at),
            Some(MarkerEvent::GaveUp { sim_ns: last })
        );
        assert_eq!(c.flush(at), Some(last));
    }

    /// The world changes before the first episode's marker is read: the read follows the
    /// episode, and a result for the old one is dropped.
    #[test]
    fn a_marker_read_follows_the_current_episode() {
        let t0 = Instant::now();
        let mut c = EpisodeClock::new();
        c.on_frame(tick(5, 300.0), t0);
        let out = c.on_frame(tick(6, 0.05), wall(t0, 1));
        assert_eq!(
            out,
            FrameOutcome::default(),
            "no rule applied, nothing published"
        );
        assert_eq!(c.marker_lookup_due(wall(t0, 1)), Some(6));
        assert_eq!(c.on_marker(5, Ok(Some(1)), wall(t0, 1)), None, "stale");
        let event = c.on_marker(6, Ok(Some(70_000_000_000)), wall(t0, 1));
        assert!(matches!(
            event,
            Some(MarkerEvent::Adopted {
                epoch_ns: 70_000_000_000,
                ..
            })
        ));
        assert_eq!(c.sim_ns(0.05), nanos(0.05) + 70_000_000_000);
    }

    /// A bridge that watched the reload keeps its own epoch whatever the marker says, and
    /// says so when they differ. A marker not written yet is looked for again.
    #[test]
    fn a_watched_change_keeps_its_epoch_and_compares() {
        let t0 = Instant::now();
        let mut c = watching();
        let last = carla_secs(1000.0, 3);
        c.on_frame(tick(7, last), t0);
        let out = c.on_frame(tick(8, 0.05), wall(t0, 1));
        let own = nanos(last) + nanos(f64::from(0.05f32));
        assert!(out.publish_ns.is_some(), "a watched change never withholds");
        // csb writes the marker just after the load: the first read may find nothing.
        assert_eq!(c.marker_lookup_due(wall(t0, 1)), Some(8));
        assert_eq!(c.on_marker(8, Ok(None), wall(t0, 1)), None);
        c.on_frame(tick(8, 0.10), wall(t0, 2));
        let later = wall(t0, 1) + MARKER_RETRY;
        assert_eq!(c.marker_lookup_due(later), Some(8));
        assert_eq!(
            c.on_marker(8, Ok(Some(own + 50_000_001)), later),
            Some(MarkerEvent::Disagrees {
                own_ns: own,
                marker_ns: own + 50_000_001
            })
        );
        assert_eq!(c.sim_ns(0.10), nanos(0.10) + own, "ours is kept");
        assert_eq!(
            c.marker_lookup_due(later + MARKER_RETRY),
            None,
            "compared once"
        );

        // Next reload: the marker agrees.
        c.on_frame(tick(9, 0.0), wall(t0, 3));
        let epoch = own + nanos(0.10) + nanos(f64::from(0.05f32));
        assert_eq!(
            c.on_marker(9, Ok(Some(epoch)), wall(t0, 3)),
            Some(MarkerEvent::Agrees { epoch_ns: epoch })
        );
    }

    /// No marker after a watched change, three times over: stop looking, keep publishing.
    #[test]
    fn a_missing_marker_after_a_watched_change_is_given_up_quietly() {
        let t0 = Instant::now();
        let mut c = watching();
        c.on_frame(tick(1, 100.0), t0);
        c.on_frame(tick(2, 0.0), wall(t0, 1));
        let mut at = wall(t0, 1);
        for _ in 0..MARKER_ATTEMPTS {
            assert_eq!(c.marker_lookup_due(at), Some(2));
            assert_eq!(c.on_marker(2, Ok(None), at), None);
            at += MARKER_RETRY;
            c.on_frame(tick(2, 1.0), at);
        }
        assert_eq!(c.marker_lookup_due(at + MARKER_RETRY), None);
    }

    /// A reconnect on which no frame had arrived before is a join, not a watched change.
    #[test]
    fn a_first_frame_after_a_reconnect_with_no_history_is_a_join() {
        let t0 = Instant::now();
        let mut c = EpisodeClock::new();
        c.mark_reconnect();
        assert_eq!(c.on_frame(tick(3, 200.0), t0), FrameOutcome::default());
        assert!(matches!(
            c.on_marker(3, Ok(Some(5)), t0),
            Some(MarkerEvent::Adopted { epoch_ns: 5, .. })
        ));
    }

    /// The marker's `role_name`: shared with csb's `epoch_marker` tests.
    const MARKER_CASES: &[(&str, Option<i64>)] = &[
        ("epoch_ns:150000002", Some(150_000_002)),
        ("epoch_ns:0", Some(0)),
        ("epoch_ns:5819510766462", Some(5_819_510_766_462)),
        ("epoch_ns:", None),
        ("epoch_ns:-1", None),
        ("epoch_ns:+1", None),
        ("epoch_ns:1.5", None),
        ("epoch_ns: 1", None),
        ("epoch_ns:99999999999999999999", None),
        ("default", None),
        ("hero", None),
        ("EPOCH_NS:1", None),
    ];

    #[test]
    fn marker_role_names_parse_like_csb_writes_them() {
        for &(role, epoch) in MARKER_CASES {
            assert_eq!(parse_epoch_marker(role), epoch, "{role:?}");
        }
        assert_eq!(
            parse_epoch_marker(&format!("{EPOCH_MARKER_PREFIX}{}", 150_000_002)),
            Some(150_000_002)
        );
    }

    /// Several markers (a csb that could not see the old one): the newest actor wins;
    /// actors of the blueprint that are not markers are ignored.
    #[test]
    fn the_newest_marker_wins() {
        assert_eq!(epoch_from_markers(Vec::<(u32, &str)>::new()), None);
        assert_eq!(epoch_from_markers([(40, "default")]), None);
        assert_eq!(
            epoch_from_markers([
                (41, "epoch_ns:7"),
                (55, "epoch_ns:9"),
                (60, "default"),
                (50, "epoch_ns:8")
            ]),
            Some(9)
        );
    }
}
