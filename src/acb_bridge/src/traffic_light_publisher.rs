//! Traffic signal state from CARLA's light actors, as Autoware's V2X input
//! (carla-scenario-bridge roadmap 015, "Signal state reaches Autoware from CARLA's light
//! actors, published by acb").
//!
//! Autoware's `traffic_light_arbiter` takes two inputs: its own camera recognition and
//! `external/traffic_signals`. This publishes the second from what CARLA's lights actually
//! show, so the ego learns signal state from the simulator in any ROS domain, without SSv2.
//! Two publishers on the topic would interleave: SSv2's own (`publish_conventional_traffic_
//! signals`) must be off wherever this is on.
//!
//! Which CARLA light is which Lanelet2 signal is not worked out here. carla_scenario_bridge
//! resolves it at every Initialize and writes `traffic_lights.resolved.yaml` beside the map
//! (`resolved_signals.rs` there); this reads that file from `traffic_light_map_path` (empty =
//! off, as is `none`) and reads it again whenever its mtime changes. Each group is one Lanelet2
//! regulatory element, which is what Autoware's `TrafficLightGroup` id means.
//!
//! It runs on a CARLA client and `on_tick` subscription of its own, like the clock, and from
//! node start rather than hero attach: Autoware's topic monitor on the arbiter's output
//! times out between scenarios otherwise (/clock keeps running then). Every message is
//! stamped with its frame's simulation time through [`SimClock::stamp`], the same
//! nanoseconds as that frame's `/clock`. Lights beyond `ground_truth_range_m` of the hero
//! are left out; with no hero there is no centre, and every mapped light is published.

use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc, Arc,
    },
    time::{Duration, Instant, SystemTime},
};

use autoware_perception_msgs::msg::{
    TrafficLightElement, TrafficLightGroup, TrafficLightGroupArray,
};
use carla::{
    client::{ActorBase, ActorKind, TrafficLight, World, WorldSnapshot},
    rpc::TrafficLightState,
};
use serde::Deserialize;

use crate::clock::SimClock;

/// Autoware's V2X input to `traffic_light_arbiter`.
pub const TOPIC: &str = "/perception/traffic_light_recognition/external/traffic_signals";

/// At most one message per this much *simulation* time. A scenario steps 50 ms per frame, so
/// every scenario frame is published; CARLA free-running between scenarios steps ~20 ms, so
/// about every other frame is. A wall-clock cap would drop scenario frames instead: SSv2's
/// frames often arrive in pairs ~17 ms apart.
const MIN_PUBLISH_SIM_S: f64 = 0.04;

/// Whether a frame at simulation time `now` (CARLA elapsed seconds) is due, given the last
/// published one. Time going backwards is a new episode: due.
fn publish_due(last: Option<f64>, now: f64) -> bool {
    last.is_none_or(|l| !(0.0..MIN_PUBLISH_SIM_S).contains(&(now - l)))
}
/// How often the file's mtime is looked at, and a missing hero looked for.
const RECHECK: Duration = Duration::from_secs(1);
/// No frame for this long: ask whether the world was replaced or the server is gone.
const STALL_CHECK: Duration = Duration::from_secs(2);
const MAX_FAILURES: u32 = 3;

/// One entry of the resolved table, as far as this bridge needs it.
#[derive(Debug, Deserialize)]
struct FileSignal {
    #[serde(default)]
    regulatory_element_ids: Vec<i64>,
    opendrive_id: String,
    #[serde(default)]
    carla_position: Option<[f64; 3]>,
}

#[derive(Debug, Deserialize)]
struct FileTable {
    #[serde(default)]
    signals: Vec<FileSignal>,
}

/// A CARLA light the table maps.
#[derive(Debug, Clone, PartialEq)]
pub struct MappedLight {
    pub opendrive_id: String,
    /// CARLA frame, metres; `None` if the table did not know it (never range-filtered).
    pub position: Option<[f64; 3]>,
}

/// The resolved table, grouped the way Autoware wants it.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct LightMap {
    /// Distinct CARLA lights, by OpenDRIVE id.
    pub lights: Vec<MappedLight>,
    /// Regulatory element id to indices into `lights`.
    pub groups: BTreeMap<i64, Vec<usize>>,
    /// Table entries without a regulatory element: nothing in Autoware can name them.
    pub skipped: usize,
}

impl LightMap {
    pub fn parse(yaml: &str) -> Result<Self, serde_yaml::Error> {
        let table: FileTable = serde_yaml::from_str(yaml)?;
        let mut map = LightMap::default();
        for s in table.signals {
            if s.regulatory_element_ids.is_empty() {
                map.skipped += 1;
                continue;
            }
            let index = match map
                .lights
                .iter()
                .position(|l| l.opendrive_id == s.opendrive_id)
            {
                Some(i) => i,
                None => {
                    map.lights.push(MappedLight {
                        opendrive_id: s.opendrive_id.clone(),
                        position: s.carla_position,
                    });
                    map.lights.len() - 1
                }
            };
            for reg in s.regulatory_element_ids {
                let members = map.groups.entry(reg).or_default();
                if !members.contains(&index) {
                    members.push(index);
                }
            }
        }
        Ok(map)
    }
}

/// A light's colour as Autoware sees it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Colour {
    // Ordered least to most restrictive; see `group_colour`.
    Unknown,
    Green,
    Amber,
    Red,
}

pub fn colour_of(state: TrafficLightState) -> Colour {
    match state {
        TrafficLightState::Red => Colour::Red,
        TrafficLightState::Yellow => Colour::Amber,
        TrafficLightState::Green => Colour::Green,
        // Off and Unknown: CARLA shows nothing, so there is nothing to report.
        _ => Colour::Unknown,
    }
}

/// The element Autoware gets for a colour: a solid circle, at full confidence, because this
/// is the simulator's own state rather than a recognition.
pub fn element(colour: Colour) -> TrafficLightElement {
    let (color, shape, status, confidence) = match colour {
        Colour::Red => (
            TrafficLightElement::RED,
            TrafficLightElement::CIRCLE,
            TrafficLightElement::SOLID_ON,
            1.0,
        ),
        Colour::Amber => (
            TrafficLightElement::AMBER,
            TrafficLightElement::CIRCLE,
            TrafficLightElement::SOLID_ON,
            1.0,
        ),
        Colour::Green => (
            TrafficLightElement::GREEN,
            TrafficLightElement::CIRCLE,
            TrafficLightElement::SOLID_ON,
            1.0,
        ),
        Colour::Unknown => (
            TrafficLightElement::UNKNOWN,
            TrafficLightElement::UNKNOWN,
            TrafficLightElement::UNKNOWN,
            0.0,
        ),
    };
    TrafficLightElement {
        color,
        shape,
        status,
        confidence,
    }
}

/// One colour for a regulatory element whose lights disagree: the most restrictive known
/// one (a mapping mistake must not show a green through a red), `Unknown` only if every
/// light is.
pub fn group_colour(colours: impl IntoIterator<Item = Colour>) -> Colour {
    colours.into_iter().max().unwrap_or(Colour::Unknown)
}

/// Whether a light is within `range_m` of the hero (planar). No hero, or no known position
/// for the light: in range.
pub fn in_range(light: Option<[f64; 3]>, hero: Option<[f64; 2]>, range_m: f64) -> bool {
    match (light, hero) {
        (Some(l), Some(h)) => ((l[0] - h[0]).powi(2) + (l[1] - h[1]).powi(2)).sqrt() <= range_m,
        _ => true,
    }
}

/// The groups for one frame. `colours[i]` is light `i`'s colour, `None` if it could not be
/// read; a group with no readable light in range is left out.
pub fn build_groups(
    map: &LightMap,
    colours: &[Option<Colour>],
    hero: Option<[f64; 2]>,
    range_m: f64,
) -> Vec<TrafficLightGroup> {
    map.groups
        .iter()
        .filter_map(|(&reg, members)| {
            let seen: Vec<Colour> = members
                .iter()
                .filter(|&&i| in_range(map.lights[i].position, hero, range_m))
                .filter_map(|&i| colours.get(i).copied().flatten())
                .collect();
            if seen.is_empty() {
                return None;
            }
            Some(TrafficLightGroup {
                traffic_light_group_id: reg,
                elements: vec![element(group_colour(seen))],
                predictions: Vec::new(),
            })
        })
        .collect()
}

pub struct Config {
    pub address: String,
    pub port: u16,
    pub map_path: PathBuf,
    pub vehicle_name: String,
    pub range_m: f64,
}

/// Start the publisher thread, or `None` (logged) when `map_path` is empty.
pub fn spawn(
    node: &rclrs::Node,
    clock: SimClock,
    config: Config,
    running: Arc<AtomicBool>,
) -> crate::error::Result<Option<std::thread::JoinHandle<()>>> {
    if config.map_path.as_os_str().is_empty() || config.map_path.as_os_str() == "none" {
        tracing::info!(
            "Traffic signals from CARLA: off (traffic_light_map_path is empty or \"none\"). \
             Nothing from this bridge on {TOPIC}"
        );
        return Ok(None);
    }
    let publisher = Arc::new(node.create_publisher::<TrafficLightGroupArray>(TOPIC)?);
    let range_m = if config.range_m.is_finite() && config.range_m > 0.0 {
        config.range_m
    } else {
        100.0
    };
    tracing::info!(
        "Traffic signals from CARLA: publishing {TOPIC} from {} (range {range_m} m around \
         '{}'). SSv2's own V2X publisher must be off, or two sources interleave",
        config.map_path.display(),
        config.vehicle_name
    );
    let handle = std::thread::Builder::new()
        .name("acb-signals".into())
        .spawn(move || {
            let mut worker = Worker {
                stats: Stats::default(),
                last_colours: Vec::new(),
                publisher,
                clock,
                range_m,
                config,
                file: FileWatch::default(),
                lights: Vec::new(),
                lights_for: None,
                hero: None,
                hero_checked: None,
                last_publish: None,
            };
            while running.load(Ordering::SeqCst) {
                let client = match carla::client::Client::connect(
                    &worker.config.address,
                    worker.config.port,
                    None,
                ) {
                    Ok(mut c) => {
                        let _ = c.set_timeout(Duration::from_secs(10));
                        c
                    }
                    Err(e) => {
                        tracing::debug!("signals: CARLA not reachable ({e}); retrying");
                        std::thread::sleep(Duration::from_secs(2));
                        continue;
                    }
                };
                worker.follow(&client, &running);
                worker.lights_for = None; // handles belong to the connection
            }
        })
        .expect("spawn the traffic signal thread");
    Ok(Some(handle))
}

/// The table file and when it was last read.
#[derive(Default)]
struct FileWatch {
    mtime: Option<SystemTime>,
    map: LightMap,
    /// Bumped on every successful (re)load, so the actor cache knows to refresh.
    generation: u64,
    checked: Option<Instant>,
    missing_reported: bool,
}

impl FileWatch {
    fn refresh(&mut self, path: &Path) {
        if self.checked.is_some_and(|t| t.elapsed() < RECHECK) {
            return;
        }
        self.checked = Some(Instant::now());
        let mtime = match std::fs::metadata(path).and_then(|m| m.modified()) {
            Ok(t) => t,
            Err(e) => {
                if !self.missing_reported {
                    self.missing_reported = true;
                    tracing::warn!(
                        "signals: cannot read {} ({e}); publishing nothing until \
                         carla_scenario_bridge writes it at the next Initialize",
                        path.display()
                    );
                }
                return;
            }
        };
        if self.mtime == Some(mtime) {
            return;
        }
        match std::fs::read_to_string(path)
            .map_err(|e| e.to_string())
            .and_then(|t| LightMap::parse(&t).map_err(|e| e.to_string()))
        {
            Ok(map) => {
                self.missing_reported = false;
                self.mtime = Some(mtime);
                self.generation += 1;
                tracing::info!(
                    "signals: loaded {} ({} light(s) in {} regulatory element(s){})",
                    path.display(),
                    map.lights.len(),
                    map.groups.len(),
                    if map.skipped > 0 {
                        format!("; {} without a regulatory element skipped", map.skipped)
                    } else {
                        String::new()
                    }
                );
                self.map = map;
            }
            // Keep the previous table: a half-written or broken file must not blank the
            // signals mid-scenario. csb writes atomically, so this is a hand edit. Remember
            // the mtime so it is reported once per change, not once a second.
            Err(e) => {
                self.mtime = Some(mtime);
                tracing::warn!(
                    "signals: {} unreadable ({e}); keeping the last table",
                    path.display()
                );
            }
        }
    }
}

/// What the publisher did, summarised in the log every [`STATS_EVERY`].
#[derive(Default)]
struct Stats {
    since: Option<Instant>,
    messages: usize,
    empty: usize,
    state_errors: usize,
    without_hero: usize,
    last_error: Option<String>,
    last_hero: Option<[f64; 2]>,
}

const STATS_EVERY: Duration = Duration::from_secs(30);

impl Stats {
    fn note(&mut self, groups: usize, errors: usize, hero: Option<[f64; 2]>) {
        self.since.get_or_insert_with(Instant::now);
        self.messages += 1;
        self.empty += usize::from(groups == 0);
        self.state_errors += errors;
        self.without_hero += usize::from(hero.is_none());
        if hero.is_some() {
            self.last_hero = hero;
        }
    }

    fn report_if_due(&mut self) {
        if self.since.is_none_or(|t| t.elapsed() < STATS_EVERY) {
            return;
        }
        tracing::info!(
            "signals: {} message(s) in {:.0} s, {} with no group, {} without a hero; {} light \
             state read error(s){}; hero last at {:?}",
            self.messages,
            self.since.map_or(0.0, |t| t.elapsed().as_secs_f64()),
            self.empty,
            self.without_hero,
            self.state_errors,
            self.last_error
                .as_deref()
                .map(|e| format!(" (last: {e})"))
                .unwrap_or_default(),
            self.last_hero
        );
        *self = Stats::default();
    }
}

struct Worker {
    stats: Stats,
    /// Last colour read per light (aligned with `lights`), held over a failed read.
    last_colours: Vec<Option<Colour>>,
    publisher: Arc<rclrs::Publisher<TrafficLightGroupArray>>,
    clock: SimClock,
    range_m: f64,
    config: Config,
    file: FileWatch,
    /// Actor handles, aligned with `file.map.lights`.
    lights: Vec<Option<TrafficLight>>,
    /// (episode, file generation) the handles were resolved for.
    lights_for: Option<(u64, u64)>,
    hero: Option<u32>,
    hero_checked: Option<Instant>,
    /// CARLA elapsed seconds of the last published frame.
    last_publish: Option<f64>,
}

impl Worker {
    /// Subscribe to frames on `client` and publish until the connection looks dead.
    fn follow(&mut self, client: &carla::client::Client, running: &AtomicBool) {
        let Ok(mut world) = client.world() else {
            std::thread::sleep(Duration::from_secs(2));
            return;
        };
        let (tx, rx) = mpsc::sync_channel::<WorldSnapshot>(4);
        let tx_cb = tx.clone();
        let Ok(mut cb) = world.on_tick(move |s| {
            let _ = tx_cb.try_send(s); // full: the worker is behind, the next frame will do
        }) else {
            std::thread::sleep(Duration::from_secs(2));
            return;
        };
        let mut episode = world.id().unwrap_or(0);
        let mut failures = 0u32;
        let mut last_frame = Instant::now();
        let mut last_check = Instant::now();
        while running.load(Ordering::SeqCst) {
            match rx.recv_timeout(Duration::from_millis(200)) {
                Ok(mut snapshot) => {
                    while let Ok(newer) = rx.try_recv() {
                        snapshot = newer;
                    }
                    last_frame = Instant::now();
                    failures = 0;
                    self.on_frame(&world, &snapshot);
                }
                Err(mpsc::RecvTimeoutError::Timeout) => {
                    if last_frame.elapsed() < STALL_CHECK || last_check.elapsed() < STALL_CHECK {
                        continue;
                    }
                    last_check = Instant::now();
                    match client.world().and_then(|w| w.id().map(|id| (w, id))) {
                        Ok((_, id)) if id == episode => failures = 0,
                        Ok((mut new_world, id)) => {
                            let _ = world.remove_on_tick(cb);
                            let tx_cb = tx.clone();
                            match new_world.on_tick(move |s| {
                                let _ = tx_cb.try_send(s);
                            }) {
                                Ok(new_cb) => {
                                    tracing::info!(
                                        "signals: the world was replaced; following episode {id}"
                                    );
                                    cb = new_cb;
                                    world = new_world;
                                    episode = id;
                                }
                                Err(_) => return,
                            }
                        }
                        Err(e) => {
                            failures += 1;
                            if failures >= MAX_FAILURES {
                                tracing::warn!(
                                    "signals: CARLA stopped answering ({e}); reconnecting"
                                );
                                let _ = world.remove_on_tick(cb);
                                return;
                            }
                        }
                    }
                }
                Err(mpsc::RecvTimeoutError::Disconnected) => break,
            }
        }
        let _ = world.remove_on_tick(cb);
    }

    fn on_frame(&mut self, world: &World, snapshot: &WorldSnapshot) {
        self.file.refresh(&self.config.map_path);
        if self.file.map.lights.is_empty() {
            return;
        }
        let elapsed = snapshot.timestamp().elapsed_seconds;
        if !publish_due(self.last_publish, elapsed) {
            return;
        }
        let key = (snapshot.id(), self.file.generation);
        if self.lights_for != Some(key) {
            self.resolve_lights(world);
            self.last_colours.clear();
            self.lights_for = Some(key);
            self.hero = None;
            self.hero_checked = None;
        }
        let hero = self.hero_position(world, snapshot);
        // A light whose state cannot be read this frame keeps the colour it last showed:
        // dropping its group instead would make the arbiter see the signal blink out.
        let mut errors = 0usize;
        self.last_colours.resize(self.lights.len(), None);
        for (light, last) in self.lights.iter().zip(self.last_colours.iter_mut()) {
            let Some(light) = light else {
                *last = None;
                continue;
            };
            match light.state() {
                Ok(s) => *last = Some(colour_of(s)),
                Err(e) => {
                    errors += 1;
                    self.stats.last_error = Some(e.to_string());
                }
            }
        }
        let colours = self.last_colours.clone();
        let groups = build_groups(&self.file.map, &colours, hero, self.range_m);
        self.stats.note(groups.len(), errors, hero);
        let msg = TrafficLightGroupArray {
            stamp: self.clock.stamp(elapsed),
            traffic_light_groups: groups,
        };
        if let Err(e) = self.publisher.publish(msg) {
            tracing::warn!("signals: publish failed: {e}");
        }
        self.last_publish = Some(elapsed);
        self.stats.report_if_due();
    }

    /// Look up every mapped light's actor in the current episode.
    fn resolve_lights(&mut self, world: &World) {
        let mut missing = Vec::new();
        self.lights = self
            .file
            .map
            .lights
            .iter()
            .map(|l| {
                let light = world
                    .traffic_light_from_open_drive(&l.opendrive_id)
                    .ok()
                    .flatten()
                    .and_then(|a| match a.into_kinds() {
                        ActorKind::TrafficLight(t) => Some(t),
                        _ => None,
                    });
                if light.is_none() {
                    missing.push(l.opendrive_id.clone());
                }
                light
            })
            .collect();
        if missing.is_empty() {
            tracing::info!("signals: {} CARLA light(s) resolved", self.lights.len());
        } else {
            tracing::warn!(
                "signals: {} of {} CARLA light(s) resolved; no actor for OpenDRIVE id(s) {:?} \
                 (a table from another town?)",
                self.lights.len() - missing.len(),
                self.lights.len(),
                missing
            );
        }
    }

    /// The hero's planar position in this frame, looking for it at most once per `RECHECK`.
    fn hero_position(&mut self, world: &World, snapshot: &WorldSnapshot) -> Option<[f64; 2]> {
        if let Some(id) = self.hero {
            if let Some(s) = snapshot.find(id) {
                let l = s.transform().location;
                return Some([l.x as f64, l.y as f64]);
            }
            self.hero = None; // gone
        }
        if self.hero_checked.is_some_and(|t| t.elapsed() < RECHECK) {
            return None;
        }
        self.hero_checked = Some(Instant::now());
        let vehicles = world.actors().ok()?.filter("vehicle.*").ok()?;
        for actor in vehicles.iter() {
            let named = actor.attributes().is_ok_and(|attrs| {
                attrs
                    .iter()
                    .any(|a| a.id() == "role_name" && a.value_string() == self.config.vehicle_name)
            });
            if named {
                self.hero = Some(actor.id());
                let l = snapshot.find(actor.id())?.transform().location;
                return Some([l.x as f64, l.y as f64]);
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TABLE: &str = r#"
# Written by carla_scenario_bridge
town: Town01
lanelet2_map: /m/Town01/lanelet2_map.osm
signals:
- way_id: 99
  regulatory_element_ids: []
  opendrive_id: '77'
  carla_position: null
- way_id: 43760
  regulatory_element_ids:
  - 43856
  opendrive_id: '13'
  carla_position:
  - 3.0
  - -4.0
  - 0.5
- way_id: 43763
  regulatory_element_ids:
  - 43856
  opendrive_id: '12'
  carla_position:
  - 1.0
  - -2.0
  - 0.5
- way_id: 500
  regulatory_element_ids:
  - 501
  - 502
  opendrive_id: '12'
  carla_position:
  - 1.0
  - -2.0
  - 0.5
unmapped:
  lanelet_way_ids: []
  carla_opendrive_ids:
  - '40'
"#;

    #[test]
    fn the_table_groups_lights_by_regulatory_element() {
        let map = LightMap::parse(TABLE).unwrap();
        assert_eq!(map.skipped, 1, "the entry without a regulatory element");
        assert_eq!(
            map.lights,
            vec![
                MappedLight {
                    opendrive_id: "13".into(),
                    position: Some([3.0, -4.0, 0.5])
                },
                MappedLight {
                    opendrive_id: "12".into(),
                    position: Some([1.0, -2.0, 0.5])
                },
            ],
            "one handle per CARLA light, however many ways map to it"
        );
        assert_eq!(map.groups.get(&43856), Some(&vec![0, 1]));
        assert_eq!(map.groups.get(&501), Some(&vec![1]));
        assert_eq!(map.groups.get(&502), Some(&vec![1]));
    }

    #[test]
    fn a_table_without_signals_is_empty_and_garbage_is_an_error() {
        assert_eq!(
            LightMap::parse("town: Town01\n").unwrap(),
            LightMap::default()
        );
        assert!(LightMap::parse("signals: [ {").is_err());
    }

    #[test]
    fn carla_colours_map_to_solid_circles() {
        let e = element(colour_of(TrafficLightState::Red));
        assert_eq!(
            (e.color, e.shape, e.status, e.confidence),
            (
                TrafficLightElement::RED,
                TrafficLightElement::CIRCLE,
                TrafficLightElement::SOLID_ON,
                1.0
            )
        );
        assert_eq!(
            element(colour_of(TrafficLightState::Yellow)).color,
            TrafficLightElement::AMBER
        );
        assert_eq!(
            element(colour_of(TrafficLightState::Green)).color,
            TrafficLightElement::GREEN
        );
        for s in [TrafficLightState::Off, TrafficLightState::Unknown] {
            let name = format!("{s:?}");
            let e = element(colour_of(s));
            assert_eq!(
                (e.color, e.confidence),
                (TrafficLightElement::UNKNOWN, 0.0),
                "{name}"
            );
        }
    }

    #[test]
    fn disagreeing_lights_report_the_most_restrictive_known_colour() {
        use Colour::*;
        assert_eq!(group_colour([Green, Red]), Red);
        assert_eq!(group_colour([Green, Amber]), Amber);
        assert_eq!(group_colour([Unknown, Green]), Green);
        assert_eq!(group_colour([Unknown]), Unknown);
        assert_eq!(group_colour([]), Unknown);
    }

    #[test]
    fn groups_follow_the_range_and_skip_unreadable_lights() {
        let map = LightMap::parse(TABLE).unwrap();
        let colours = [Some(Colour::Green), Some(Colour::Red)];
        let ids = |g: &[TrafficLightGroup]| {
            g.iter()
                .map(|g| g.traffic_light_group_id)
                .collect::<Vec<_>>()
        };

        // No hero: everything, and 43856 is red because one of its lights is.
        let all = build_groups(&map, &colours, None, 100.0);
        assert_eq!(ids(&all), vec![501, 502, 43856]);
        assert_eq!(all[2].elements[0].color, TrafficLightElement::RED);

        // Hero next to light 13 only (light 12 is 2.83 m further): 43856 shows 13's green.
        let near13 = build_groups(&map, &colours, Some([4.0, -5.0]), 2.0);
        assert_eq!(ids(&near13), vec![43856]);
        assert_eq!(near13[0].elements[0].color, TrafficLightElement::GREEN);

        // Far from everything.
        assert!(build_groups(&map, &colours, Some([1000.0, 0.0]), 100.0).is_empty());

        // Light 12 unreadable: its groups disappear, 43856 falls back to 13.
        let partial = build_groups(&map, &[Some(Colour::Green), None], None, 100.0);
        assert_eq!(ids(&partial), vec![43856]);
        assert_eq!(partial[0].elements[0].color, TrafficLightElement::GREEN);
    }

    #[test]
    fn every_scenario_frame_is_due_and_free_running_frames_are_thinned() {
        assert!(publish_due(None, 10.0));
        assert!(publish_due(Some(10.0), 10.05), "a 50 ms scenario frame");
        assert!(
            !publish_due(Some(10.0), 10.021),
            "a 21 ms free-running frame"
        );
        assert!(publish_due(Some(10.0), 10.042));
        assert!(
            publish_due(Some(10.0), 3.0),
            "a new episode restarts elapsed"
        );
    }

    #[test]
    fn a_light_without_a_position_is_never_filtered_out() {
        assert!(in_range(None, Some([1e6, 1e6]), 1.0));
        assert!(in_range(Some([0.0, 0.0, 9.0]), None, 1.0));
        assert!(in_range(Some([3.0, 4.0, 0.0]), Some([0.0, 0.0]), 5.0));
        assert!(!in_range(Some([3.0, 4.0, 0.0]), Some([0.0, 0.0]), 4.99));
    }

    #[test]
    fn the_file_is_reread_only_when_its_mtime_changes() {
        let dir = std::env::temp_dir().join(format!("acb_signals_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("traffic_lights.resolved.yaml");
        let mut watch = FileWatch::default();

        watch.refresh(&path); // missing: nothing loaded, no panic
        assert_eq!(watch.generation, 0);

        std::fs::write(&path, TABLE).unwrap();
        watch.checked = None;
        watch.refresh(&path);
        assert_eq!((watch.generation, watch.map.groups.len()), (1, 3));

        watch.checked = None;
        watch.refresh(&path); // same mtime: not re-read
        assert_eq!(watch.generation, 1);

        // A broken rewrite keeps the last good table.
        std::fs::write(&path, "signals: [ {").unwrap();
        let later = SystemTime::now() + Duration::from_secs(5);
        std::fs::File::options()
            .write(true)
            .open(&path)
            .unwrap()
            .set_modified(later)
            .unwrap();
        watch.checked = None;
        watch.refresh(&path);
        assert_eq!((watch.generation, watch.map.groups.len()), (1, 3));

        let _ = std::fs::remove_dir_all(&dir);
    }
}
