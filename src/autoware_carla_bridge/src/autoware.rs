use std::{collections::HashMap, sync::Arc};

use arc_swap::ArcSwap;
use carla::client::ActorBase;
use rclrs::IntoPrimitiveOptions;

use crate::{
    autoware_detection::AutowareDetector,
    bridge::sensor_bridge::SensorType,
    carla_vehicle::CarlaVehicle,
    coordinate_conversion,
    error::{BridgeError, Result},
    tf_bridge::TFBuffer,
};
use std::sync::Mutex;

/// Main Autoware ROS communication coordinator
///
/// Manages Autoware detection, ROS topic names, sensor configurations,
/// and coordinate transforms. This is the central hub for all Autoware-related
/// ROS communication.
pub struct Autoware {
    // === ROS Infrastructure ===
    /// ROS node handle
    /// NOTE: Kept for future use in creating additional ROS resources dynamically
    #[allow(dead_code)]
    node: rclrs::Node,

    // === Autoware Detection and Configuration ===
    /// Autoware instance detector (monitors /robot_description)
    detector: AutowareDetector,

    /// TF buffer for sensor transforms
    tf_buffer: TFBuffer,

    // === Pose Publishing Configuration ===
    /// Whether to publish pose directly to /localization/kinematic_state
    /// Ground truth is always published to /carla/ground_truth/* for debug/evaluation
    publish_direct_localization: bool,

    /// CARLA vehicle reference for getting ground truth pose
    vehicle: Option<Arc<Mutex<CarlaVehicle>>>,

    // === Ground Truth Publishers (Always Active) ===
    /// Publisher for ground truth pose (always published for debug/evaluation)
    /// Following AWSIM convention - we provide sensor data and let Autoware's
    /// localization compute the pose. This is always published for debugging/evaluation.
    pub_ground_truth: Arc<rclrs::Publisher<nav_msgs::msg::Odometry>>,

    /// Publisher for ground truth TF transforms (always published for debug/evaluation)
    pub_ground_truth_tf: Arc<rclrs::Publisher<tf2_msgs::msg::TFMessage>>,

    // === Direct Localization Publishers (Optional) ===
    /// Publisher for localization pose (bypasses Autoware localization when enabled)
    pub_localization: Option<Arc<rclrs::Publisher<nav_msgs::msg::Odometry>>>,

    /// Publisher for TF transforms (bypasses Autoware TF when enabled)
    pub_tf: Option<Arc<rclrs::Publisher<tf2_msgs::msg::TFMessage>>>,

    // === Vehicle Status Publishers ===
    // NOTE: These publishers are created but not yet used. They will be used in future phases
    // to publish vehicle status from CARLA to Autoware.
    #[allow(dead_code)]
    pub_actuation_status: Arc<rclrs::Publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>>,
    #[allow(dead_code)]
    pub_steering_status: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::SteeringReport>>,
    #[allow(dead_code)]
    pub_gear_status: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::GearReport>>,
    #[allow(dead_code)]
    pub_control_mode: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>>,
    #[allow(dead_code)]
    pub_turn_indicators_status:
        Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>>,
    #[allow(dead_code)]
    pub_hazard_lights_status: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>>,

    // === Vehicle Command Subscriptions ===
    _sub_actuation_cmd: Arc<rclrs::Subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>>,
    _sub_gear_cmd: Arc<rclrs::Subscription<autoware_vehicle_msgs::msg::GearCommand>>,
    _sub_gate_mode: Arc<rclrs::Subscription<tier4_control_msgs::msg::GateMode>>,
    _sub_turn_indicators_cmd:
        Arc<rclrs::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>>,
    _sub_hazard_lights_cmd:
        Arc<rclrs::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>>,

    // === Shared State for Commands ===
    /// NOTE: Command state fields kept for future vehicle control integration
    #[allow(dead_code)]
    current_actuation_cmd: Arc<ArcSwap<tier4_vehicle_msgs::msg::ActuationCommandStamped>>,
    #[allow(dead_code)]
    current_gear_cmd: Arc<ArcSwap<autoware_vehicle_msgs::msg::GearCommand>>,
    #[allow(dead_code)]
    current_gate_mode: Arc<ArcSwap<tier4_control_msgs::msg::GateMode>>,
    #[allow(dead_code)]
    current_turn_indicators_cmd: Arc<ArcSwap<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>>,
    #[allow(dead_code)]
    current_hazard_lights_cmd: Arc<ArcSwap<autoware_vehicle_msgs::msg::HazardLightsCommand>>,

    // === Initial Pose (Modern Autoware API) ===
    /// Initial pose for vehicle spawning (from modern Autoware localization API)
    /// Set from /localization/kinematic_state when initialization_state becomes 3
    initial_pose: Arc<Mutex<Option<nalgebra::Isometry3<f32>>>>,

    // === Localization State Monitoring ===
    /// Localization initialization state (from /localization/initialization_state)
    /// State values: 0=UNKNOWN, 1=UNINITIALIZED, 2=INITIALIZING, 3=INITIALIZED
    /// NOTE: Updated by callback, reserved for future Phase 4+ features
    #[allow(dead_code)]
    localization_init_state: Arc<Mutex<u16>>,

    /// Current kinematic state from localization (from /localization/kinematic_state)
    /// NOTE: Updated by callback, reserved for future Phase 4+ features
    #[allow(dead_code)]
    localization_kinematic_state: Arc<Mutex<Option<nav_msgs::msg::Odometry>>>,

    /// Subscription to /localization/initialization_state (kept alive)
    _localization_init_state_sub:
        Arc<rclrs::Subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>>,

    /// Subscription to /localization/kinematic_state (kept alive)
    _localization_kinematic_state_sub: Arc<rclrs::Subscription<nav_msgs::msg::Odometry>>,

    // === Topic Configuration (existing - kept for compatibility) ===
    // Vehicle publish topic (root namespace)
    pub topic_actuation_status: String,
    pub topic_velocity_status: String,
    pub topic_steering_status: String,
    pub topic_gear_status: String,
    pub topic_control_mode: String,
    pub topic_turn_indicators_status: String,
    pub topic_hazard_lights_status: String,
    // Vehicle subscribe topic (root namespace)
    pub topic_actuation_cmd: String,
    pub topic_gear_cmd: String,
    pub topic_current_gate_mode: String,
    pub topic_turn_indicators_cmd: String,
    pub topic_hazard_lights_cmd: String,
    // Sensor publish topic (dynamically built from sensor names)
    pub list_image_raw: HashMap<String, String>,
    pub list_camera_info: HashMap<String, String>,
    pub list_lidar: HashMap<String, String>,
    pub list_lidar_semantics: HashMap<String, String>,
    pub list_gnss: HashMap<String, String>,
    pub list_imu: HashMap<String, String>,
}

impl Autoware {
    /// Create new Autoware instance with ROS node
    ///
    /// Initializes AutowareDetector to monitor for Autoware instances,
    /// creates TFBuffer for transform management, and sets up standard
    /// Autoware topic names. Does NOT wait for Autoware detection -
    /// call `wait_for_detection()` for that.
    ///
    /// After creation, call `set_vehicle()` to enable ground truth publishing.
    ///
    /// # Arguments
    /// * `node` - ROS node handle for subscriptions and publishers
    /// * `publish_direct_localization` - Whether to publish directly to /localization/kinematic_state
    ///   Ground truth is always published to /carla/ground_truth/*
    ///
    /// # Returns
    /// Result containing Autoware instance or error
    pub fn new(node: rclrs::Node, publish_direct_localization: bool) -> Result<Self> {
        tracing::info!("Initializing Autoware coordinator...");
        tracing::info!(
            "Direct localization publishing: {}",
            if publish_direct_localization {
                "enabled"
            } else {
                "disabled"
            }
        );

        // Create AutowareDetector (monitors /robot_description)
        // NOTE: Long health timeout (1 hour) because /robot_description is a latched topic
        // that's only published once at startup, not a continuous heartbeat.
        let detector = AutowareDetector::new(
            node.clone(),
            None,                                       // detection_timeout: use default (60s)
            Some(std::time::Duration::from_secs(3600)), // health_timeout: 1 hour
        )?;

        // Create TFBuffer (subscribes to /tf_static)
        let tf_buffer = TFBuffer::new(node.clone())?;

        // Create ground truth publishers (always published for debug/evaluation)
        // Following AWSIM convention: we provide sensor data and let Autoware's localization
        // compute the pose. Ground truth is always published for debugging and evaluation.
        let pub_ground_truth =
            Arc::new(node.create_publisher::<nav_msgs::msg::Odometry>(
                "/carla/ground_truth/odom".reliable(),
            )?);

        let pub_ground_truth_tf = Arc::new(
            node.create_publisher::<tf2_msgs::msg::TFMessage>("/carla/ground_truth/tf".reliable())?,
        );

        tracing::info!("Ground truth publishers created (always active):");
        tracing::info!("  - /carla/ground_truth/odom (Odometry)");
        tracing::info!("  - /carla/ground_truth/tf (TF)");

        // Create direct localization publishers if enabled
        let (pub_localization, pub_tf) = if publish_direct_localization {
            tracing::info!(
                "Creating direct localization publishers (bypasses Autoware localization):"
            );
            let loc = Arc::new(node.create_publisher::<nav_msgs::msg::Odometry>(
                "/localization/kinematic_state".reliable(),
            )?);
            let tf = Arc::new(node.create_publisher::<tf2_msgs::msg::TFMessage>("/tf".reliable())?);
            tracing::info!("  - /localization/kinematic_state (Odometry)");
            tracing::info!("  - /tf (TF)");
            (Some(loc), Some(tf))
        } else {
            tracing::info!("Autoware localization will compute pose from sensor data");
            (None, None)
        };

        // Create vehicle status publishers
        let pub_actuation_status = Arc::new(
            node.create_publisher::<tier4_vehicle_msgs::msg::ActuationStatusStamped>(
                "vehicle/status/actuation_status".reliable(),
            )?,
        );

        let pub_steering_status = Arc::new(
            node.create_publisher::<autoware_vehicle_msgs::msg::SteeringReport>(
                "vehicle/status/steering_status".reliable(),
            )?,
        );

        let pub_gear_status = Arc::new(
            node.create_publisher::<autoware_vehicle_msgs::msg::GearReport>(
                "vehicle/status/gear_status".reliable(),
            )?,
        );

        let pub_control_mode = Arc::new(
            node.create_publisher::<autoware_vehicle_msgs::msg::ControlModeReport>(
                "vehicle/status/control_mode".reliable(),
            )?,
        );

        let pub_turn_indicators_status = Arc::new(
            node.create_publisher::<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
                "vehicle/status/turn_indicators_status".reliable(),
            )?,
        );

        let pub_hazard_lights_status = Arc::new(
            node.create_publisher::<autoware_vehicle_msgs::msg::HazardLightsReport>(
                "vehicle/status/hazard_lights_status".reliable(),
            )?,
        );

        // Create shared state for vehicle commands
        let current_actuation_cmd = Arc::new(ArcSwap::new(Arc::new(
            tier4_vehicle_msgs::msg::ActuationCommandStamped::default(),
        )));
        let current_gear_cmd = Arc::new(ArcSwap::new(Arc::new(
            autoware_vehicle_msgs::msg::GearCommand::default(),
        )));
        let current_gate_mode = Arc::new(ArcSwap::new(Arc::new(
            tier4_control_msgs::msg::GateMode::default(),
        )));
        let current_turn_indicators_cmd = Arc::new(ArcSwap::new(Arc::new(
            autoware_vehicle_msgs::msg::TurnIndicatorsCommand::default(),
        )));
        let current_hazard_lights_cmd = Arc::new(ArcSwap::new(Arc::new(
            autoware_vehicle_msgs::msg::HazardLightsCommand::default(),
        )));

        // Create vehicle command subscriptions
        let actuation_cmd_cb = current_actuation_cmd.clone();
        let sub_actuation_cmd = Arc::new(
            node.create_subscription::<tier4_vehicle_msgs::msg::ActuationCommandStamped, _>(
                "control/command/actuation_cmd".reliable(),
                move |msg: tier4_vehicle_msgs::msg::ActuationCommandStamped| {
                    actuation_cmd_cb.store(Arc::new(msg));
                },
            )?,
        );

        let gear_cmd_cb = current_gear_cmd.clone();
        let sub_gear_cmd = Arc::new(
            node.create_subscription::<autoware_vehicle_msgs::msg::GearCommand, _>(
                "control/command/gear_cmd".reliable(),
                move |msg: autoware_vehicle_msgs::msg::GearCommand| {
                    gear_cmd_cb.store(Arc::new(msg));
                },
            )?,
        );

        let gate_mode_cb = current_gate_mode.clone();
        let sub_gate_mode = Arc::new(
            node.create_subscription::<tier4_control_msgs::msg::GateMode, _>(
                "control/current_gate_mode".reliable(),
                move |msg: tier4_control_msgs::msg::GateMode| {
                    gate_mode_cb.store(Arc::new(msg));
                },
            )?,
        );

        let turn_indicators_cmd_cb = current_turn_indicators_cmd.clone();
        let sub_turn_indicators_cmd = Arc::new(
            node.create_subscription::<autoware_vehicle_msgs::msg::TurnIndicatorsCommand, _>(
                "control/command/turn_indicators_cmd".reliable(),
                move |msg: autoware_vehicle_msgs::msg::TurnIndicatorsCommand| {
                    turn_indicators_cmd_cb.store(Arc::new(msg));
                },
            )?,
        );

        let hazard_lights_cmd_cb = current_hazard_lights_cmd.clone();
        let sub_hazard_lights_cmd = Arc::new(
            node.create_subscription::<autoware_vehicle_msgs::msg::HazardLightsCommand, _>(
                "control/command/hazard_lights_cmd".reliable(),
                move |msg: autoware_vehicle_msgs::msg::HazardLightsCommand| {
                    hazard_lights_cmd_cb.store(Arc::new(msg));
                },
            )?,
        );

        // Create initial pose state (set via modern Autoware localization API)
        let initial_pose = Arc::new(Mutex::new(None));

        // Subscribe to Autoware localization initialization state
        let localization_init_state = Arc::new(Mutex::new(0u16)); // 0 = UNKNOWN
        let localization_init_state_cb = localization_init_state.clone();
        let initial_pose_from_localization = initial_pose.clone();
        let localization_kinematic_state: Arc<Mutex<Option<nav_msgs::msg::Odometry>>> =
            Arc::new(Mutex::new(None));
        let localization_kinematic_state_for_init = localization_kinematic_state.clone();

        let localization_init_state_sub = Arc::new(
            node.create_subscription::<autoware_adapi_v1_msgs::msg::LocalizationInitializationState, _>(
                "/localization/initialization_state".reliable().keep_last(1),
                move |msg: autoware_adapi_v1_msgs::msg::LocalizationInitializationState| {
                    let old_state = *localization_init_state_cb.lock().unwrap();
                    let new_state = msg.state;
                    *localization_init_state_cb.lock().unwrap() = new_state;

                    // When localization becomes INITIALIZED (state == 3), use kinematic state as initial pose
                    if old_state != 3 && new_state == 3 {
                        tracing::info!("Localization initialized (state changed from {} to 3)", old_state);

                        // Get current kinematic state and use it as initial pose
                        if let Some(kinematic_state) = localization_kinematic_state_for_init.lock().unwrap().as_ref() {
                            let pose = &kinematic_state.pose.pose;
                            tracing::info!(
                                "Using localization kinematic state as initial pose: ({:.2}, {:.2}, {:.2})",
                                pose.position.x,
                                pose.position.y,
                                pose.position.z
                            );

                            // Convert ROS pose to CARLA transform
                            let carla_isometry = crate::coordinate_conversion::ros_pose_to_carla_isometry(pose);
                            *initial_pose_from_localization.lock().unwrap() = Some(carla_isometry);
                        } else {
                            tracing::warn!("Localization initialized but no kinematic state available yet");
                        }
                    }
                },
            )?,
        );

        // Subscribe to kinematic state
        let localization_kinematic_state_cb = localization_kinematic_state.clone();
        let localization_kinematic_state_sub =
            Arc::new(node.create_subscription::<nav_msgs::msg::Odometry, _>(
                "/localization/kinematic_state".reliable().keep_last(1),
                move |msg: nav_msgs::msg::Odometry| {
                    *localization_kinematic_state_cb.lock().unwrap() = Some(msg);
                },
            )?);

        tracing::info!("Autoware coordinator initialized");

        Ok(Autoware {
            // === ROS Infrastructure ===
            node,

            // === Autoware Detection and Configuration ===
            detector,
            tf_buffer,

            // === Pose Publishing Configuration ===
            publish_direct_localization,
            vehicle: None,

            // === Ground Truth Publishers (Always Active) ===
            pub_ground_truth,
            pub_ground_truth_tf,

            // === Direct Localization Publishers (Optional) ===
            pub_localization,
            pub_tf,

            // === Vehicle Status Publishers ===
            pub_actuation_status,
            pub_steering_status,
            pub_gear_status,
            pub_control_mode,
            pub_turn_indicators_status,
            pub_hazard_lights_status,

            // === Vehicle Command Subscriptions ===
            _sub_actuation_cmd: sub_actuation_cmd,
            _sub_gear_cmd: sub_gear_cmd,
            _sub_gate_mode: sub_gate_mode,
            _sub_turn_indicators_cmd: sub_turn_indicators_cmd,
            _sub_hazard_lights_cmd: sub_hazard_lights_cmd,

            // === Shared State for Commands ===
            current_actuation_cmd,
            current_gear_cmd,
            current_gate_mode,
            current_turn_indicators_cmd,
            current_hazard_lights_cmd,

            // === Initial Pose ===
            initial_pose,

            // === Localization State Monitoring ===
            localization_init_state,
            localization_kinematic_state,
            _localization_init_state_sub: localization_init_state_sub,
            _localization_kinematic_state_sub: localization_kinematic_state_sub,

            // === Topic Configuration ===
            // Vehicle publish topic (standard Autoware topics - no prefix for namespace flexibility)
            topic_actuation_status: "vehicle/status/actuation_status".to_string(),
            topic_velocity_status: "vehicle/status/velocity_status".to_string(),
            topic_steering_status: "vehicle/status/steering_status".to_string(),
            topic_gear_status: "vehicle/status/gear_status".to_string(),
            topic_control_mode: "vehicle/status/control_mode".to_string(),
            topic_turn_indicators_status: "vehicle/status/turn_indicators_status".to_string(),
            topic_hazard_lights_status: "vehicle/status/hazard_lights_status".to_string(),
            // Vehicle subscribe topic (standard Autoware topics - no prefix for namespace flexibility)
            topic_actuation_cmd: "control/command/actuation_cmd".to_string(),
            topic_gear_cmd: "control/command/gear_cmd".to_string(),
            topic_current_gate_mode: "control/current_gate_mode".to_string(),
            topic_turn_indicators_cmd: "control/command/turn_indicators_cmd".to_string(),
            topic_hazard_lights_cmd: "control/command/hazard_lights_cmd".to_string(),
            // Sensor publish topic (will be populated dynamically)
            list_image_raw: HashMap::new(),
            list_camera_info: HashMap::new(),
            list_lidar: HashMap::new(),
            list_lidar_semantics: HashMap::new(),
            list_gnss: HashMap::new(),
            list_imu: HashMap::new(),
        })
    }

    /// Register sensor topics for Autoware topic name mapping
    ///
    /// Must be called for each sensor before creating sensor bridges.
    /// Maps sensor names to their corresponding Autoware topic paths.
    pub fn add_sensors(&mut self, sensor_type: SensorType, sensor_name: String) {
        // Standard Autoware sensor topic patterns (no prefix for namespace flexibility)
        match sensor_type {
            SensorType::CameraRgb => {
                let raw_key = format!("sensing/camera/{sensor_name}/image_raw");
                let info_key = format!("sensing/camera/{sensor_name}/camera_info");
                self.list_image_raw.insert(sensor_name.clone(), raw_key);
                self.list_camera_info.insert(sensor_name, info_key);
            }
            SensorType::Collision => {}
            SensorType::Imu => {
                let imu_key = format!("sensing/imu/{sensor_name}/imu_raw");
                self.list_imu.insert(sensor_name.clone(), imu_key);
            }
            SensorType::LidarRayCast => {
                // Publish to pointcloud_before_sync to feed into Autoware's pointcloud_preprocessor
                // This bypasses velodyne drivers (not needed for simulation)
                let lidar_key = format!("sensing/lidar/{sensor_name}/pointcloud_before_sync");
                self.list_lidar.insert(sensor_name.clone(), lidar_key);
            }
            SensorType::LidarRayCastSemantic => {
                // Publish to pointcloud_before_sync to feed into Autoware's pointcloud_preprocessor
                let lidar_key = format!("sensing/lidar/{sensor_name}/pointcloud_before_sync");
                self.list_lidar_semantics
                    .insert(sensor_name.clone(), lidar_key);
            }
            SensorType::Gnss => {
                let gnss_key = format!("sensing/gnss/{sensor_name}/nav_sat_fix");
                self.list_gnss.insert(sensor_name.clone(), gnss_key);
            }
            SensorType::NotSupport => {}
        }
    }

    pub fn get_sensors_key(
        &self,
        sensor_type: SensorType,
        sensor_name: &str,
    ) -> Option<Vec<String>> {
        match sensor_type {
            SensorType::CameraRgb => {
                let raw_key = self.list_image_raw.get(sensor_name);
                let info_key = self.list_camera_info.get(sensor_name);
                if let (Some(raw_key), Some(info_key)) = (raw_key, info_key) {
                    return Some(vec![raw_key.to_owned(), info_key.to_owned()]);
                }
            }
            SensorType::Collision => {}
            SensorType::Imu => {
                let imu_key = self.list_imu.get(sensor_name);
                if let Some(imu_key) = imu_key {
                    return Some(vec![imu_key.to_owned()]);
                }
            }
            SensorType::LidarRayCast => {
                let lidar_key = self.list_lidar.get(sensor_name);
                if let Some(lidar_key) = lidar_key {
                    return Some(vec![lidar_key.to_owned()]);
                }
            }
            SensorType::LidarRayCastSemantic => {
                let lidar_key = self.list_lidar_semantics.get(sensor_name);
                if let Some(lidar_key) = lidar_key {
                    return Some(vec![lidar_key.to_owned()]);
                }
            }
            SensorType::Gnss => {
                let gnss_key = self.list_gnss.get(sensor_name);
                if let Some(gnss_key) = gnss_key {
                    return Some(vec![gnss_key.to_owned()]);
                }
            }
            SensorType::NotSupport => {}
        };
        None
    }

    // === Autoware Detection Methods ===

    /// Wait for Autoware instance to be detected
    ///
    /// Blocks until `/robot_description` is received or timeout occurs.
    /// Verifies robot_state_publisher node and /tf_static topic.
    ///
    /// # Returns
    /// Result indicating success or timeout error
    ///
    /// NOTE: Alternative to is_alive() check, kept for explicit detection workflows
    #[allow(dead_code)]
    pub fn wait_for_detection(&self) -> Result<()> {
        tracing::info!("Waiting for Autoware detection...");
        self.detector.wait_for_detection()?;
        tracing::info!("Autoware detected successfully!");
        Ok(())
    }

    /// Check if Autoware is alive
    ///
    /// Returns true if Autoware was detected and hasn't been lost.
    pub fn is_alive(&self) -> bool {
        self.detector.is_alive()
    }

    /// Get URDF content from Autoware
    ///
    /// Returns the latest URDF XML received from `/robot_description`.
    ///
    /// # Returns
    /// Option containing URDF string if available
    #[allow(dead_code)]
    pub fn get_urdf(&self) -> Option<String> {
        self.detector.get_urdf()
    }

    /// Perform health check on Autoware connection
    ///
    /// Updates detection state based on recent activity.
    /// Should be called periodically in main loop.
    pub fn health_check(&self) {
        self.detector.health_check()
    }

    /// Get diagnostic information about Autoware connection
    ///
    /// # Returns
    /// Diagnostic information including state and health
    ///
    /// NOTE: Diagnostic API kept for monitoring and debugging purposes
    #[allow(dead_code)]
    pub fn get_diagnostics(&self) -> crate::autoware_detection::DetectionDiagnostics {
        self.detector.get_diagnostics()
    }

    /// Get TF buffer for transform lookups
    ///
    /// Provides access to the TF buffer for looking up sensor transforms.
    ///
    /// # Returns
    /// Reference to TFBuffer
    pub fn get_tf_buffer(&self) -> &TFBuffer {
        &self.tf_buffer
    }

    /// Set the CARLA vehicle reference
    ///
    /// Must be called after vehicle spawn to enable ground truth publishing
    /// and localization initialization from CARLA vehicle pose.
    ///
    /// # Arguments
    /// * `vehicle` - Arc<Mutex<CarlaVehicle>> for shared access
    pub fn set_vehicle(&mut self, vehicle: Arc<Mutex<CarlaVehicle>>) {
        tracing::info!("Vehicle reference set for Autoware coordinator");
        self.vehicle = Some(vehicle);
    }

    /// Main tick function - call once per frame
    ///
    /// Handles:
    /// 1. Getting vehicle pose/velocity from CARLA
    /// 2. Converting coordinates to ROS
    /// 3. Publishing ground truth
    /// 4. Handling localization initialization/re-initialization
    ///
    /// # Arguments
    /// * `sim_time` - CARLA simulation time in seconds
    ///
    /// # Returns
    /// Result indicating success or error
    pub fn tick(&self, sim_time: f64) -> Result<()> {
        // Get vehicle from stored reference
        let vehicle_arc = self.vehicle.as_ref().ok_or_else(|| {
            BridgeError::AutowareIssue("Vehicle not set - call set_vehicle() first".to_string())
        })?;

        let vehicle_guard = vehicle_arc.lock().unwrap();
        let vehicle = vehicle_guard.get_vehicle();

        // Get CARLA transform/velocity
        let transform = vehicle.transform()?;
        let velocity = vehicle.velocity()?;
        let angular_velocity = vehicle.angular_velocity()?;

        // Convert to nalgebra for coordinate conversion
        let na_transform = transform.to_na();
        let na_velocity = velocity.to_na();
        let na_angular_velocity = angular_velocity.to_na();

        // Convert CARLA coordinates to ROS coordinates
        let position = coordinate_conversion::carla_to_ros_position(&nalgebra::Vector3::new(
            na_transform.translation.x as f64,
            na_transform.translation.y as f64,
            na_transform.translation.z as f64,
        ));

        // Convert CARLA rotation (quaternion) to ROS quaternion
        let carla_quat = nalgebra::Quaternion::new(
            na_transform.rotation.w as f64,
            na_transform.rotation.i as f64,
            na_transform.rotation.j as f64,
            na_transform.rotation.k as f64,
        );
        let (roll, pitch, yaw) = coordinate_conversion::quaternion_to_euler(&carla_quat);
        let orientation = coordinate_conversion::euler_to_quaternion(
            -roll, // Roll sign flip for ROS right-handed system
            pitch, -yaw, // Yaw sign flip for ROS right-handed system
        );

        let linear_vel = coordinate_conversion::carla_to_ros_velocity(&nalgebra::Vector3::new(
            na_velocity.x as f64,
            na_velocity.y as f64,
            na_velocity.z as f64,
        ));

        let angular_vel =
            coordinate_conversion::carla_to_ros_angular_velocity(&nalgebra::Vector3::new(
                na_angular_velocity.x as f64,
                na_angular_velocity.y as f64,
                na_angular_velocity.z as f64,
            ));

        // Create ROS timestamp
        let ros_timestamp = builtin_interfaces::msg::Time {
            sec: sim_time as i32,
            nanosec: ((sim_time - sim_time.floor()) * 1e9) as u32,
        };

        // Publish ground truth
        self.publish_ground_truth(
            &ros_timestamp,
            &[position.x, position.y, position.z],
            &[orientation.w, orientation.i, orientation.j, orientation.k],
            &[linear_vel.x, linear_vel.y, linear_vel.z],
            &[angular_vel.x, angular_vel.y, angular_vel.z],
        )?;

        Ok(())
    }

    // === Initial Pose Methods ===

    /// Check if initial pose has been received
    ///
    /// Returns true if a pose has been received from /localization/kinematic_state
    /// when localization state becomes INITIALIZED (3).
    ///
    /// # Returns
    /// Boolean indicating if initial pose is available
    ///
    /// NOTE: Reserved for future Phase 4+ features
    #[allow(dead_code)]
    pub fn has_initial_pose(&self) -> bool {
        self.initial_pose.lock().unwrap().is_some()
    }

    /// Get the initial pose
    ///
    /// Returns a copy of the initial pose if available.
    ///
    /// # Returns
    /// Result containing the initial pose or error if not available
    ///
    /// NOTE: Alternative to take_initial_pose(), kept for non-consuming access
    #[allow(dead_code)]
    pub fn get_initial_pose(&self) -> Result<nalgebra::Isometry3<f32>> {
        self.initial_pose.lock().unwrap().ok_or_else(|| {
            crate::error::BridgeError::AutowareIssue("No initial pose available".to_string())
        })
    }

    /// Take the initial pose (consuming it)
    ///
    /// Returns the initial pose and clears it from the struct.
    /// Useful for one-time vehicle spawning.
    ///
    /// # Returns
    /// Result containing the initial pose or error if not available
    ///
    /// NOTE: Reserved for future Phase 4+ features
    #[allow(dead_code)]
    pub fn take_initial_pose(&self) -> Result<nalgebra::Isometry3<f32>> {
        self.initial_pose.lock().unwrap().take().ok_or_else(|| {
            crate::error::BridgeError::AutowareIssue("No initial pose available".to_string())
        })
    }

    /// Get ROS node handle
    ///
    /// Provides access to the ROS node for creating additional
    /// publishers or subscriptions.
    ///
    /// # Returns
    /// Clone of ROS node handle (cheap - uses Arc internally)
    ///
    /// NOTE: Public API method kept for dynamic ROS resource creation
    #[allow(dead_code)]
    pub fn node(&self) -> rclrs::Node {
        self.node.clone()
    }

    // === Vehicle Command Accessors ===
    // NOTE: Command accessors kept for future vehicle control integration

    /// Get current actuation command
    ///
    /// Returns the latest actuation command received from Autoware.
    ///
    /// # Returns
    /// Arc containing the latest ActuationCommandStamped
    #[allow(dead_code)]
    pub fn get_actuation_cmd(&self) -> Arc<tier4_vehicle_msgs::msg::ActuationCommandStamped> {
        self.current_actuation_cmd.load_full()
    }

    /// Get current gear command
    ///
    /// Returns the latest gear command received from Autoware.
    ///
    /// # Returns
    /// Arc containing the latest GearCommand
    #[allow(dead_code)]
    pub fn get_gear_cmd(&self) -> Arc<autoware_vehicle_msgs::msg::GearCommand> {
        self.current_gear_cmd.load_full()
    }

    /// Get current gate mode
    ///
    /// Returns the latest gate mode received from Autoware.
    ///
    /// # Returns
    /// Arc containing the latest GateMode
    #[allow(dead_code)]
    pub fn get_gate_mode(&self) -> Arc<tier4_control_msgs::msg::GateMode> {
        self.current_gate_mode.load_full()
    }

    /// Get current turn indicators command
    ///
    /// Returns the latest turn indicators command received from Autoware.
    ///
    /// # Returns
    /// Arc containing the latest TurnIndicatorsCommand
    #[allow(dead_code)]
    pub fn get_turn_indicators_cmd(
        &self,
    ) -> Arc<autoware_vehicle_msgs::msg::TurnIndicatorsCommand> {
        self.current_turn_indicators_cmd.load_full()
    }

    /// Get current hazard lights command
    ///
    /// Returns the latest hazard lights command received from Autoware.
    ///
    /// # Returns
    /// Arc containing the latest HazardLightsCommand
    #[allow(dead_code)]
    pub fn get_hazard_lights_cmd(&self) -> Arc<autoware_vehicle_msgs::msg::HazardLightsCommand> {
        self.current_hazard_lights_cmd.load_full()
    }

    // === Vehicle Status Publishing ===

    /// Publish ground truth pose (for debugging/evaluation only)
    ///
    /// Publishes the vehicle's ground truth pose and twist from CARLA.
    /// The publishing behavior depends on the pose_publishing_mode:
    ///
    /// - Direct mode: Publishes to `/localization/kinematic_state` and `/tf` (main localization topics)
    /// - GroundTruth mode: Publishes to `/carla/ground_truth/odom` and `/carla/ground_truth/tf` (debug topics)
    ///
    /// In GroundTruth mode, following AWSIM convention: we provide sensor data (LiDAR, GNSS, IMU)
    /// and let Autoware's localization module (NDT + EKF) compute the vehicle pose from those sensors.
    ///
    /// # Arguments
    /// * `timestamp` - ROS timestamp for the message
    /// * `position` - Vehicle position (x, y, z) in map frame (meters)
    /// * `orientation` - Vehicle orientation quaternion (w, x, y, z)
    /// * `linear_velocity` - Linear velocity (x, y, z) in m/s
    /// * `angular_velocity` - Angular velocity (x, y, z) in rad/s
    ///
    /// # Returns
    /// Result indicating success or error
    pub fn publish_ground_truth(
        &self,
        timestamp: &builtin_interfaces::msg::Time,
        position: &[f64; 3],
        orientation: &[f64; 4],
        linear_velocity: &[f64; 3],
        angular_velocity: &[f64; 3],
    ) -> Result<()> {
        // Create Odometry message
        let mut odom = nav_msgs::msg::Odometry::default();
        odom.header.stamp = timestamp.clone();
        odom.header.frame_id = "map".to_string();
        odom.child_frame_id = "base_link".to_string();

        // Set pose
        odom.pose.pose.position.x = position[0];
        odom.pose.pose.position.y = position[1];
        odom.pose.pose.position.z = position[2];
        odom.pose.pose.orientation.w = orientation[0];
        odom.pose.pose.orientation.x = orientation[1];
        odom.pose.pose.orientation.y = orientation[2];
        odom.pose.pose.orientation.z = orientation[3];

        // Set twist
        odom.twist.twist.linear.x = linear_velocity[0];
        odom.twist.twist.linear.y = linear_velocity[1];
        odom.twist.twist.linear.z = linear_velocity[2];
        odom.twist.twist.angular.x = angular_velocity[0];
        odom.twist.twist.angular.y = angular_velocity[1];
        odom.twist.twist.angular.z = angular_velocity[2];

        // Create TF message
        let mut tf_msg = tf2_msgs::msg::TFMessage::default();
        let mut transform = geometry_msgs::msg::TransformStamped::default();
        transform.header.stamp = timestamp.clone();
        transform.header.frame_id = "map".to_string();
        transform.child_frame_id = "base_link".to_string();
        transform.transform.translation.x = position[0];
        transform.transform.translation.y = position[1];
        transform.transform.translation.z = position[2];
        transform.transform.rotation.w = orientation[0];
        transform.transform.rotation.x = orientation[1];
        transform.transform.rotation.y = orientation[2];
        transform.transform.rotation.z = orientation[3];
        tf_msg.transforms.push(transform);

        // Always publish ground truth (for debug/evaluation)
        self.pub_ground_truth.publish(&odom)?;
        self.pub_ground_truth_tf.publish(&tf_msg)?;

        // Also publish to direct localization topics if enabled
        if self.publish_direct_localization {
            if let Some(pub_loc) = &self.pub_localization {
                pub_loc.publish(&odom)?;
            }
            if let Some(pub_tf) = &self.pub_tf {
                pub_tf.publish(&tf_msg)?;
            }
        }

        Ok(())
    }

    /// Check if localization has been initialized (from /localization/initialization_state)
    ///
    /// Returns true if state == 3 (INITIALIZED)
    #[allow(dead_code)]
    pub fn is_localization_initialized(&self) -> bool {
        *self.localization_init_state.lock().unwrap() == 3
    }
}
