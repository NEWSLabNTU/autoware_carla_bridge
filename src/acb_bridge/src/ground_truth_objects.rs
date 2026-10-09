//! Publish CARLA's actors as Autoware perception output, bypassing the perception stack.
//!
//! Autoware normally derives objects from the LiDAR, which is the realistic path and the
//! default. This exists for the two cases where that is in the way: debugging planning or
//! control without perception in the loop, and iterating on a host whose GPU is busy --
//! `lidar_detection_model: clustering` is already the CPU fallback here because CARLA takes
//! what GPU headroom there is.
//!
//! It publishes `PredictedObjects` on `/perception/object_recognition/objects`, which is
//! what planning consumes. That topic is normally written by `map_based_prediction`, so this
//! must only be enabled with Autoware's perception stack switched off
//! (`perception:=false` in `carla_simulator.launch.xml`). Two publishers on one topic is the
//! shape of bug this repository has already paid for twice.
//!
//! Ground truth is not free of lies: it reports every actor the server knows about,
//! including ones no sensor could see -- through buildings, behind the ego, at any range.
//! Planning that looks good on this may not survive real perception.

use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
};

use carla::client::{ActorBase, World, WorldSnapshot};

use crate::{coordinate_conversion, error::Result};

/// Horizon of the constant-velocity prediction, and the spacing of its samples.
///
/// `map_based_prediction` normally fills `predicted_paths`, and planning modules iterate
/// them; an object with none reads as one with no future. Constant velocity is the honest
/// prediction to make from a single frame of ground truth -- it says the actor keeps doing
/// what it is doing, and claims nothing about intent.
const PREDICTION_HORIZON_S: f64 = 5.0;
const PREDICTION_STEP_S: f64 = 0.5;

/// Below this speed CARLA's own velocity reading is treated as "none" and the pose history
/// is used instead. A physics-off actor reads exactly 0.000 (roadmap 014, gap 5), so any
/// small positive threshold separates the two cases.
const CARLA_VELOCITY_EPS_MPS: f64 = 0.01;

/// Frames closer together than this are the same frame seen twice; keep the last estimate.
const MIN_DT_S: f64 = 1e-4;
/// A gap longer than this is not one motion any more -- the actor was out of the snapshot,
/// or the stream stalled. Start the history again rather than average across it.
const MAX_DT_S: f64 = 1.0;
/// Faster than any vehicle in a scenario (250 km/h): a jump implying more is a teleport --
/// a spawn, a respawn, SSv2 re-placing an entity -- not motion.
const MAX_SPEED_MPS: f64 = 70.0;
/// A half turn in a quarter second. Walkers can turn about that fast; anything faster is
/// a re-placement, not a turn.
const MAX_YAW_RATE_RPS: f64 = 4.0 * std::f64::consts::PI;

/// One observation of an actor: ROS world axes (x forward/east, y left, z up; metres), yaw
/// in radians counter-clockwise, `t` in CARLA simulation seconds.
#[derive(Clone, Copy, Debug)]
struct PoseSample {
    t: f64,
    pos: nalgebra::Vector3<f64>,
    yaw: f64,
}

/// An object's twist the way `PredictedObject` carries it: linear velocity in the object's
/// own frame (x forward, y left, z up) and yaw rate counter-clockwise, all ROS, SI units.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct ObjectTwist {
    longitudinal: f64,
    lateral: f64,
    vertical: f64,
    yaw_rate: f64,
}

impl ObjectTwist {
    /// The planar velocity in world axes, for an object currently heading `yaw`.
    fn world_xy(&self, yaw: f64) -> (f64, f64) {
        let (s, c) = yaw.sin_cos();
        (
            self.longitudinal * c - self.lateral * s,
            self.longitudinal * s + self.lateral * c,
        )
    }
}

struct Track {
    type_id: String,
    last: PoseSample,
    twist: ObjectTwist,
}

/// Velocity of each actor from its pose across CARLA frames.
///
/// Exists because CARLA does not know how fast a teleported actor moves. SSv2 drives NPCs
/// by `set_transform` with physics off, and CARLA then reports `get_velocity() = 0.000`
/// for every one of them (measured, roadmap 014 "NPC motion (gap 5)"). Published as is,
/// every NPC reads as parked and Autoware predicts it will stay where it is -- a cut-in
/// is invisible until it has happened.
///
/// Timestamps are CARLA's `elapsed_seconds`, never the wall clock: the bridge can process
/// frames late or skip some (tick_follower), and a wall-clock dt would turn that jitter
/// into speed.
#[derive(Default)]
struct PoseVelocityEstimator {
    tracks: HashMap<u32, Track>,
}

impl PoseVelocityEstimator {
    /// Record `sample` for actor `id` and return its twist. Zero on first sight, and after
    /// anything that breaks the history (type change on id reuse, time running backwards
    /// on a server restart, a long gap, a jump no vehicle could make).
    fn observe(&mut self, id: u32, type_id: &str, sample: PoseSample) -> ObjectTwist {
        let Some(track) = self.tracks.get_mut(&id).filter(|t| t.type_id == type_id) else {
            self.tracks.insert(
                id,
                Track {
                    type_id: type_id.to_string(),
                    last: sample,
                    twist: ObjectTwist::default(),
                },
            );
            return ObjectTwist::default();
        };

        let dt = sample.t - track.last.t;
        if (0.0..MIN_DT_S).contains(&dt) {
            return track.twist;
        }
        let twist = if dt < 0.0 || dt > MAX_DT_S {
            None
        } else {
            Self::difference(&track.last, &sample, dt)
        };
        track.last = sample;
        track.twist = twist.unwrap_or_default();
        track.twist
    }

    /// Twist from two samples `dt` apart, or `None` if the step is a teleport.
    ///
    /// The displacement is resolved in the frame at the *mid* heading and scaled by
    /// `(θ/2)/sin(θ/2)`: for a body moving with constant own-frame velocity and yaw rate,
    /// the chord is exactly `R(mid) · v · dt · sin(θ/2)/(θ/2)`, so this recovers `v` with no
    /// lag and no false lateral component in a steady turn. Resolving at the new heading
    /// instead would read a vehicle in a turn as sliding outward by `v · sin(θ/2)`.
    fn difference(prev: &PoseSample, cur: &PoseSample, dt: f64) -> Option<ObjectTwist> {
        let d = cur.pos - prev.pos;
        let dyaw = wrap_angle(cur.yaw - prev.yaw);
        if d.xy().norm() / dt > MAX_SPEED_MPS || dyaw.abs() / dt > MAX_YAW_RATE_RPS {
            return None;
        }
        let half = 0.5 * dyaw;
        let arc = if half.abs() > 1e-9 {
            half / half.sin()
        } else {
            1.0
        };
        let (s, c) = (prev.yaw + half).sin_cos();
        Some(ObjectTwist {
            longitudinal: arc * (d.x * c + d.y * s) / dt,
            lateral: arc * (-d.x * s + d.y * c) / dt,
            vertical: d.z / dt,
            yaw_rate: dyaw / dt,
        })
    }

    /// Forget actors that are gone, so a CARLA id handed out again starts a new history.
    fn retain(&mut self, alive: &HashSet<u32>) {
        self.tracks.retain(|id, _| alive.contains(id));
    }
}

/// An actor's CARLA transform as an estimator sample, in ROS axes.
fn pose_sample(tf: &carla::geom::Transform, t: f64) -> PoseSample {
    let pos = coordinate_conversion::carla_to_ros_position(&nalgebra::Vector3::new(
        tf.location.x as f64,
        tf.location.y as f64,
        tf.location.z as f64,
    ));
    let (_, _, yaw_deg) = coordinate_conversion::carla_to_ros_rotation(
        tf.rotation.roll as f64,
        tf.rotation.pitch as f64,
        tf.rotation.yaw as f64,
    );
    PoseSample {
        t,
        pos,
        yaw: yaw_deg.to_radians(),
    }
}

/// CARLA's own twist when it has one, the pose-history estimate otherwise.
///
/// The rule is "CARLA's velocity if it is not ~0". A physics-on actor (the ego, anything
/// Traffic Manager drives, a walker under its AI controller) reports its real velocity,
/// which is exact and has no one-frame lag, so it wins. A physics-off actor -- every NPC
/// SSv2 places by teleport -- reports exactly 0.000 however fast it moves (roadmap 014,
/// gap 5), so the estimate is the only information there is. The one case the rule sends
/// to the estimate without needing to, a physics-on actor at rest, has a pose that is not
/// moving either, and both answers are ~0. carla-rust offers no physics-enabled query to
/// switch on instead, and would still need this for actors whose physics is toggled.
fn choose_twist(
    carla_velocity: &carla::geom::Vector3D,
    carla_angular_velocity: &carla::geom::Vector3D,
    tf: &carla::geom::Transform,
    estimated: ObjectTwist,
) -> ObjectTwist {
    let v = nalgebra::Vector3::new(
        carla_velocity.x as f64,
        carla_velocity.y as f64,
        carla_velocity.z as f64,
    );
    if v.norm() <= CARLA_VELOCITY_EPS_MPS {
        return estimated;
    }
    // CARLA reports both in world axes. Bring them into the actor's own frame first (still
    // CARLA axes), then flip Y once. Angular velocity is in DEGREES per second
    // (docs/issues/008); ROS yaw rate is the negated z, counter-clockwise.
    let body_v = tf.rotation.inverse_rotate_vector(carla_velocity);
    let body_w = tf.rotation.inverse_rotate_vector(carla_angular_velocity);
    ObjectTwist {
        longitudinal: body_v.x as f64,
        lateral: -(body_v.y as f64),
        vertical: body_v.z as f64,
        yaw_rate: -(body_w.z as f64).to_radians(),
    }
}

/// Wrap an angle into (-π, π].
fn wrap_angle(a: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    let w = a.rem_euclid(tau);
    if w > std::f64::consts::PI {
        w - tau
    } else {
        w
    }
}

pub struct GroundTruthObjectPublisher {
    publisher: Arc<rclrs::Publisher<autoware_perception_msgs::msg::PredictedObjects>>,
    /// CARLA `role_name` of the ego, so it is not published as an obstacle to itself.
    ego_role_name: String,
    range_m: f64,
    velocities: PoseVelocityEstimator,
}

impl GroundTruthObjectPublisher {
    pub fn new(node: rclrs::Node, ego_role_name: String, range_m: f64) -> Result<Self> {
        // A non-positive or non-finite range would silently publish nothing, which looks
        // exactly like a broken bridge. Fall back rather than start in that state.
        let range_m = if range_m.is_finite() && range_m > 0.0 {
            range_m
        } else {
            tracing::warn!("ground_truth_range_m = {range_m} is not usable; using 100 m");
            100.0
        };
        let publisher = Arc::new(
            node.create_publisher::<autoware_perception_msgs::msg::PredictedObjects>(
                "/perception/object_recognition/objects",
            )?,
        );
        tracing::info!(
            "Ground-truth objects: publishing /perception/object_recognition/objects \
             (range {range_m} m, excluding role_name '{ego_role_name}'). Autoware's \
             own perception must be disabled, or two publishers will interleave on this topic."
        );
        Ok(Self {
            publisher,
            ego_role_name,
            range_m,
            velocities: PoseVelocityEstimator::default(),
        })
    }

    /// Read every vehicle and walker from CARLA and publish them as perceived objects.
    ///
    /// `snapshot` is the frame the main loop just received. Poses and velocities come from
    /// it rather than from `Actor::transform()`, so each pose is paired with the simulation
    /// time it was taken at -- the velocity estimate divides by that time.
    pub fn publish(
        &mut self,
        world: &World,
        snapshot: &WorldSnapshot,
        stamp: &builtin_interfaces::msg::Time,
    ) -> Result<()> {
        let actors = world.actors()?;
        let sim_time = snapshot.timestamp().elapsed_seconds;

        // Classify by type id before anything else. A world holds far more than traffic --
        // Town01 alone carries over a hundred `static.*` props and the spectator -- and the
        // type id is a local string, where reading attributes is an RPC per actor.
        let traffic: Vec<_> = actors
            .iter()
            .filter_map(|actor| {
                let type_id = actor.type_id();
                let label = if type_id.starts_with("vehicle.") {
                    autoware_perception_msgs::msg::ObjectClassification::CAR
                } else if type_id.starts_with("walker.") {
                    autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN
                } else {
                    return None;
                };
                Some((actor, label))
            })
            .collect();

        let ego_location = traffic
            .iter()
            .find(|(actor, _)| self.is_ego(actor))
            .and_then(|(actor, _)| snapshot.find(actor.id()))
            .map(|s| s.transform().location);

        let alive: HashSet<u32> = traffic.iter().map(|(actor, _)| actor.id()).collect();
        self.velocities.retain(&alive);

        let mut objects = Vec::new();
        for (actor, label) in traffic {
            if self.is_ego(&actor) {
                continue;
            }

            let Some(state) = snapshot.find(actor.id()) else {
                continue; // not in this frame: destroyed, or not spawned yet
            };
            let tf = state.transform();
            // Every actor feeds its history, in range or not, so one that drives into
            // range arrives with a velocity instead of reading as parked for a frame.
            let pose = pose_sample(&tf, sim_time);
            let estimated = self
                .velocities
                .observe(actor.id(), actor.type_id().as_str(), pose);
            let twist = choose_twist(&state.velocity(), &state.angular_velocity(), &tf, estimated);

            if let Some(ref e) = ego_location {
                let dx = (tf.location.x - e.x) as f64;
                let dy = (tf.location.y - e.y) as f64;
                if dx.hypot(dy) > self.range_m {
                    continue;
                }
            }
            objects.push(self.to_object(&actor, label, &tf, &pose, twist));
        }

        let msg = autoware_perception_msgs::msg::PredictedObjects {
            header: std_msgs::msg::Header {
                // The frame's time, the same as its `/clock` (roadmap 015).
                stamp: stamp.clone(),
                frame_id: "map".to_string(),
            },
            objects,
        };
        self.publisher.publish(&msg)?;
        Ok(())
    }

    /// Whether this actor is the ego, by CARLA `role_name`.
    fn is_ego(&self, actor: &carla::client::Actor) -> bool {
        actor.attributes().is_ok_and(|attrs| {
            attrs
                .iter()
                .any(|a| a.id() == "role_name" && a.value_string() == self.ego_role_name)
        })
    }

    fn to_object(
        &self,
        actor: &carla::client::Actor,
        label: u8,
        tf: &carla::geom::Transform,
        pose: &PoseSample,
        twist: ObjectTwist,
    ) -> autoware_perception_msgs::msg::PredictedObject {
        let pos = pose.pos;
        let (roll, pitch, yaw) = coordinate_conversion::carla_to_ros_rotation(
            tf.rotation.roll as f64,
            tf.rotation.pitch as f64,
            tf.rotation.yaw as f64,
        );
        let q = coordinate_conversion::euler_to_quaternion(
            roll.to_radians(),
            pitch.to_radians(),
            yaw.to_radians(),
        );
        // World-frame velocity in the ROS frame, for the prediction below.
        let (vx_ros, vy_ros) = twist.world_xy(pose.yaw);

        let bb = actor.bounding_box();

        let mut obj = autoware_perception_msgs::msg::PredictedObject::default();
        // The actor id is stable for the actor's lifetime, which is what a track id means.
        obj.object_id.uuid[0..4].copy_from_slice(&actor.id().to_le_bytes());
        obj.existence_probability = 1.0;
        obj.classification = vec![autoware_perception_msgs::msg::ObjectClassification {
            label,
            probability: 1.0,
        }];
        obj.kinematics.initial_pose_with_covariance.pose.position.x = pos.x;
        obj.kinematics.initial_pose_with_covariance.pose.position.y = pos.y;
        obj.kinematics.initial_pose_with_covariance.pose.position.z = pos.z;
        obj.kinematics
            .initial_pose_with_covariance
            .pose
            .orientation
            .x = q.i;
        obj.kinematics
            .initial_pose_with_covariance
            .pose
            .orientation
            .y = q.j;
        obj.kinematics
            .initial_pose_with_covariance
            .pose
            .orientation
            .z = q.k;
        obj.kinematics
            .initial_pose_with_covariance
            .pose
            .orientation
            .w = q.w;
        // Autoware's twist is in the object's own frame, as a tracker would produce it.
        obj.kinematics.initial_twist_with_covariance.twist.linear.x = twist.longitudinal;
        obj.kinematics.initial_twist_with_covariance.twist.linear.y = twist.lateral;
        obj.kinematics.initial_twist_with_covariance.twist.linear.z = twist.vertical;
        obj.kinematics.initial_twist_with_covariance.twist.angular.z = twist.yaw_rate;
        obj.kinematics.predicted_paths =
            vec![self.constant_velocity_path(&pos, &q, vx_ros, vy_ros)];
        obj.shape.type_ = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
        obj.shape.dimensions.x = (bb.extent.x * 2.0) as f64;
        obj.shape.dimensions.y = (bb.extent.y * 2.0) as f64;
        obj.shape.dimensions.z = (bb.extent.z * 2.0) as f64;
        obj
    }

    /// Where the actor ends up if it keeps its current velocity, sampled every
    /// `PREDICTION_STEP_S` out to `PREDICTION_HORIZON_S`.
    ///
    /// Orientation is held constant: a straight-line prediction that also turned would be
    /// asserting a curve nothing in a single frame supports.
    fn constant_velocity_path(
        &self,
        pos: &nalgebra::Vector3<f64>,
        q: &nalgebra::Quaternion<f64>,
        vx: f64,
        vy: f64,
    ) -> autoware_perception_msgs::msg::PredictedPath {
        let steps = (PREDICTION_HORIZON_S / PREDICTION_STEP_S).round() as usize;
        let path = (0..=steps)
            .map(|i| {
                let t = i as f64 * PREDICTION_STEP_S;
                let mut p = geometry_msgs::msg::Pose::default();
                p.position.x = pos.x + vx * t;
                p.position.y = pos.y + vy * t;
                p.position.z = pos.z;
                p.orientation.x = q.i;
                p.orientation.y = q.j;
                p.orientation.z = q.k;
                p.orientation.w = q.w;
                p
            })
            .collect();

        let step_nanos = (PREDICTION_STEP_S * 1e9) as u64;
        autoware_perception_msgs::msg::PredictedPath {
            path,
            time_step: builtin_interfaces::msg::Duration {
                sec: (step_nanos / 1_000_000_000) as i32,
                nanosec: (step_nanos % 1_000_000_000) as u32,
            },
            // The only prediction offered, so all the probability mass belongs to it.
            confidence: 1.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.05; // CARLA's fixed_delta_seconds in this deployment

    fn sample(t: f64, x: f64, y: f64, yaw: f64) -> PoseSample {
        PoseSample {
            t,
            pos: nalgebra::Vector3::new(x, y, 0.0),
            yaw,
        }
    }

    fn close(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() < tol, "expected {b}, got {a}");
    }

    #[test]
    fn first_sight_is_zero() {
        let mut est = PoseVelocityEstimator::default();
        let tw = est.observe(7, "vehicle.x", sample(10.0, 5.0, 5.0, 0.3));
        assert_eq!(tw, ObjectTwist::default());
    }

    #[test]
    fn constant_velocity_is_exact_in_the_object_frame() {
        // 10 m/s along a 30-degree heading: all longitudinal, none lateral.
        let mut est = PoseVelocityEstimator::default();
        let yaw = 30f64.to_radians();
        let mut tw = ObjectTwist::default();
        for i in 0..5 {
            let t = 100.0 + i as f64 * DT;
            let d = 10.0 * (t - 100.0);
            tw = est.observe(1, "vehicle.x", sample(t, d * yaw.cos(), d * yaw.sin(), yaw));
        }
        close(tw.longitudinal, 10.0, 1e-9);
        close(tw.lateral, 0.0, 1e-9);
        close(tw.yaw_rate, 0.0, 1e-9);
        let (vx, vy) = tw.world_xy(yaw);
        close(vx, 10.0 * yaw.cos(), 1e-9);
        close(vy, 10.0 * yaw.sin(), 1e-9);
    }

    #[test]
    fn sideways_walker_reads_lateral() {
        // Facing +x, stepping left (+y) at 1.2 m/s.
        let mut est = PoseVelocityEstimator::default();
        est.observe(3, "walker.x", sample(0.0, 0.0, 0.0, 0.0));
        let tw = est.observe(3, "walker.x", sample(DT, 0.0, 1.2 * DT, 0.0));
        close(tw.longitudinal, 0.0, 1e-9);
        close(tw.lateral, 1.2, 1e-9);
    }

    #[test]
    fn steady_turn_is_exact_with_no_false_lateral() {
        // 8 m/s on a 20 m radius, turning left (counter-clockwise): w = 0.4 rad/s.
        let (v, r) = (8.0, 20.0);
        let w = v / r;
        let pose = |t: f64| {
            let th = w * t;
            sample(t, r * th.sin(), r * (1.0 - th.cos()), th)
        };
        let mut est = PoseVelocityEstimator::default();
        est.observe(2, "vehicle.x", pose(0.0));
        // A coarse step on purpose: the chord correction must hold, not just the limit.
        let tw = est.observe(2, "vehicle.x", pose(0.5));
        close(tw.longitudinal, v, 1e-9);
        close(tw.lateral, 0.0, 1e-9);
        close(tw.yaw_rate, w, 1e-9);
    }

    #[test]
    fn heading_wraps_across_pi() {
        let mut est = PoseVelocityEstimator::default();
        let a = std::f64::consts::PI - 0.01;
        est.observe(4, "vehicle.x", sample(0.0, 0.0, 0.0, a));
        let tw = est.observe(4, "vehicle.x", sample(DT, 0.0, 0.0, -a));
        close(tw.yaw_rate, 0.02 / DT, 1e-9);
    }

    #[test]
    fn teleport_resets_and_the_next_frame_recovers() {
        let mut est = PoseVelocityEstimator::default();
        est.observe(5, "vehicle.x", sample(0.0, 0.0, 0.0, 0.0));
        est.observe(5, "vehicle.x", sample(DT, 0.5, 0.0, 0.0));
        // 200 m in one frame is a re-placement, not 4000 m/s.
        let tw = est.observe(5, "vehicle.x", sample(2.0 * DT, 200.0, 0.0, 0.0));
        assert_eq!(tw, ObjectTwist::default());
        let tw = est.observe(5, "vehicle.x", sample(3.0 * DT, 200.5, 0.0, 0.0));
        close(tw.longitudinal, 10.0, 1e-9);
    }

    #[test]
    fn same_frame_twice_keeps_the_estimate() {
        let mut est = PoseVelocityEstimator::default();
        est.observe(6, "vehicle.x", sample(0.0, 0.0, 0.0, 0.0));
        let a = est.observe(6, "vehicle.x", sample(DT, 0.5, 0.0, 0.0));
        let b = est.observe(6, "vehicle.x", sample(DT, 0.5, 0.0, 0.0));
        assert_eq!(a, b);
        close(b.longitudinal, 10.0, 1e-9);
    }

    #[test]
    fn time_going_backwards_or_a_long_gap_resets() {
        let mut est = PoseVelocityEstimator::default();
        est.observe(8, "vehicle.x", sample(50.0, 0.0, 0.0, 0.0));
        // Server restart rewinds elapsed_seconds.
        let tw = est.observe(8, "vehicle.x", sample(1.0, 0.5, 0.0, 0.0));
        assert_eq!(tw, ObjectTwist::default());
        let tw = est.observe(8, "vehicle.x", sample(1.0 + 5.0, 1.0, 0.0, 0.0));
        assert_eq!(tw, ObjectTwist::default());
    }

    #[test]
    fn reused_id_with_another_type_starts_over() {
        let mut est = PoseVelocityEstimator::default();
        est.observe(9, "vehicle.a", sample(0.0, 0.0, 0.0, 0.0));
        let tw = est.observe(9, "walker.b", sample(DT, 0.05, 0.0, 0.0));
        assert_eq!(tw, ObjectTwist::default());
    }

    #[test]
    fn despawned_actor_is_forgotten() {
        let mut est = PoseVelocityEstimator::default();
        est.observe(10, "vehicle.x", sample(0.0, 0.0, 0.0, 0.0));
        est.retain(&HashSet::new());
        // Same id, same type, pose next door: without retain this would read 10 m/s.
        let tw = est.observe(10, "vehicle.x", sample(DT, 0.5, 0.0, 0.0));
        assert_eq!(tw, ObjectTwist::default());
    }

    #[test]
    fn carla_velocity_wins_when_it_has_one() {
        // Physics-on actor heading CARLA yaw 90 (ROS -90: facing -y), moving at 5 m/s
        // along CARLA +y, turning at CARLA +10 deg/s (clockwise from above).
        let tf = carla::geom::Transform {
            location: carla::geom::Location::new(0.0, 0.0, 0.0),
            rotation: carla::geom::Rotation::new(0.0, 90.0, 0.0),
        };
        let v = carla::geom::Vector3D::new(0.0, 5.0, 0.0);
        let w = carla::geom::Vector3D::new(0.0, 0.0, 10.0);
        let tw = choose_twist(&v, &w, &tf, ObjectTwist::default());
        close(tw.longitudinal, 5.0, 1e-5);
        close(tw.lateral, 0.0, 1e-5);
        close(tw.yaw_rate, -10f64.to_radians(), 1e-6);
    }

    #[test]
    fn zero_carla_velocity_falls_back_to_the_estimate() {
        let tf = carla::geom::Transform {
            location: carla::geom::Location::new(0.0, 0.0, 0.0),
            rotation: carla::geom::Rotation::new(0.0, 0.0, 0.0),
        };
        let zero = carla::geom::Vector3D::new(0.0, 0.0, 0.0);
        let est = ObjectTwist {
            longitudinal: 7.0,
            lateral: 0.1,
            vertical: 0.0,
            yaw_rate: 0.2,
        };
        assert_eq!(choose_twist(&zero, &zero, &tf, est), est);
    }
}
