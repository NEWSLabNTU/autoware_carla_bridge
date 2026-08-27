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

use std::sync::Arc;

use carla::client::{ActorBase, World};

use crate::{coordinate_conversion, error::Result, utils};

/// Horizon of the constant-velocity prediction, and the spacing of its samples.
///
/// `map_based_prediction` normally fills `predicted_paths`, and planning modules iterate
/// them; an object with none reads as one with no future. Constant velocity is the honest
/// prediction to make from a single frame of ground truth -- it says the actor keeps doing
/// what it is doing, and claims nothing about intent.
const PREDICTION_HORIZON_S: f64 = 5.0;
const PREDICTION_STEP_S: f64 = 0.5;

pub struct GroundTruthObjectPublisher {
    publisher: Arc<rclrs::Publisher<autoware_perception_msgs::msg::PredictedObjects>>,
    node: rclrs::Node,
    /// CARLA `role_name` of the ego, so it is not published as an obstacle to itself.
    ego_role_name: String,
    range_m: f64,
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
            node,
            ego_role_name,
            range_m,
        })
    }

    /// Read every vehicle and walker from CARLA and publish them as perceived objects.
    pub fn publish(&self, world: &World) -> Result<()> {
        let actors = world.actors()?;

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
            .and_then(|(actor, _)| actor.transform().ok())
            .map(|t| t.location);

        let mut objects = Vec::new();
        for (actor, label) in traffic {
            if self.is_ego(&actor) {
                continue;
            }

            let Ok(tf) = actor.transform() else {
                continue; // actor destroyed between listing and reading
            };
            if let Some(ref e) = ego_location {
                let dx = (tf.location.x - e.x) as f64;
                let dy = (tf.location.y - e.y) as f64;
                if dx.hypot(dy) > self.range_m {
                    continue;
                }
            }
            objects.push(self.to_object(&actor, label, &tf));
        }

        let msg = autoware_perception_msgs::msg::PredictedObjects {
            header: std_msgs::msg::Header {
                stamp: utils::ros_time_now(&self.node),
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
    ) -> autoware_perception_msgs::msg::PredictedObject {
        let pos = coordinate_conversion::carla_to_ros_position(&nalgebra::Vector3::new(
            tf.location.x as f64,
            tf.location.y as f64,
            tf.location.z as f64,
        ));
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

        // World-frame velocity in the ROS frame, reused for the prediction below.
        let (vx_ros, vy_ros) = match actor.velocity() {
            Ok(v) => (v.x as f64, -(v.y as f64)),
            Err(_) => (0.0, 0.0),
        };

        // CARLA reports velocity in the world frame; Autoware wants it in the object's own
        // frame, which is what a tracker would have produced.
        let heading = yaw.to_radians();
        let (longitudinal, lateral) = (
            vx_ros * heading.cos() + vy_ros * heading.sin(),
            -vx_ros * heading.sin() + vy_ros * heading.cos(),
        );

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
        obj.kinematics.initial_pose_with_covariance.pose.orientation.x = q.i;
        obj.kinematics.initial_pose_with_covariance.pose.orientation.y = q.j;
        obj.kinematics.initial_pose_with_covariance.pose.orientation.z = q.k;
        obj.kinematics.initial_pose_with_covariance.pose.orientation.w = q.w;
        obj.kinematics.initial_twist_with_covariance.twist.linear.x = longitudinal;
        obj.kinematics.initial_twist_with_covariance.twist.linear.y = lateral;
        obj.kinematics.predicted_paths = vec![self.constant_velocity_path(&pos, &q, vx_ros, vy_ros)];
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
