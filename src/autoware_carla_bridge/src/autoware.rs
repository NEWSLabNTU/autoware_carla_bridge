use std::{collections::HashMap, sync::Arc};

use arc_swap::ArcSwap;
use rclrs::IntoPrimitiveOptions;

use crate::{
    autoware_detection::AutowareDetector,
    bridge::sensor_bridge::SensorType,
    error::Result,
    tf_bridge::TFBuffer,
    urdf_parser::{parse_urdf_sensors, SensorConfig},
};

/// Main Autoware ROS communication coordinator
///
/// Manages Autoware detection, ROS topic names, sensor configurations,
/// and coordinate transforms. This is the central hub for all Autoware-related
/// ROS communication.
pub struct Autoware {
    // === ROS Infrastructure ===
    /// ROS node handle
    node: rclrs::Node,

    // === Autoware Detection and Configuration ===
    /// Autoware instance detector (monitors /robot_description)
    detector: AutowareDetector,

    /// TF buffer for sensor transforms
    tf_buffer: TFBuffer,

    /// Parsed sensor configurations from URDF
    sensor_configs: Vec<SensorConfig>,

    // === Localization Publishers ===
    /// Publisher for vehicle localization (Odometry)
    pub_localization: Arc<rclrs::Publisher<nav_msgs::msg::Odometry>>,

    /// Publisher for TF transforms
    pub_tf: Arc<rclrs::Publisher<tf2_msgs::msg::TFMessage>>,

    // === Vehicle Status Publishers ===
    pub_actuation_status: Arc<rclrs::Publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>>,
    pub_velocity_status: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::VelocityReport>>,
    pub_steering_status: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::SteeringReport>>,
    pub_gear_status: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::GearReport>>,
    pub_control_mode: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>>,
    pub_turn_indicators_status:
        Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>>,
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
    current_actuation_cmd: Arc<ArcSwap<tier4_vehicle_msgs::msg::ActuationCommandStamped>>,
    current_gear_cmd: Arc<ArcSwap<autoware_vehicle_msgs::msg::GearCommand>>,
    current_gate_mode: Arc<ArcSwap<tier4_control_msgs::msg::GateMode>>,
    current_turn_indicators_cmd: Arc<ArcSwap<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>>,
    current_hazard_lights_cmd: Arc<ArcSwap<autoware_vehicle_msgs::msg::HazardLightsCommand>>,

    // === Initial Pose from RViz ===
    /// Initial pose for vehicle spawning (from /initialpose topic)
    initial_pose: Arc<std::sync::Mutex<Option<nalgebra::Isometry3<f32>>>>,

    /// Subscription to /initialpose (kept alive)
    _initialpose_sub: Arc<rclrs::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>>,

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
    /// # Arguments
    /// * `node` - ROS node handle for subscriptions and publishers
    ///
    /// # Returns
    /// Result containing Autoware instance or error
    pub fn new(node: rclrs::Node) -> Result<Self> {
        tracing::info!("Initializing Autoware coordinator...");

        // Create AutowareDetector (monitors /robot_description)
        let detector = AutowareDetector::new(node.clone(), None, None)?;

        // Create TFBuffer (subscribes to /tf_static)
        let tf_buffer = TFBuffer::new(node.clone())?;

        // Create localization publishers
        let pub_localization = Arc::new(node.create_publisher::<nav_msgs::msg::Odometry>(
            "/localization/kinematic_state".reliable(),
        )?);

        let pub_tf = Arc::new(node.create_publisher::<tf2_msgs::msg::TFMessage>("/tf".reliable())?);

        // Create vehicle status publishers
        let pub_actuation_status = Arc::new(
            node.create_publisher::<tier4_vehicle_msgs::msg::ActuationStatusStamped>(
                "vehicle/status/actuation_status".reliable(),
            )?,
        );

        let pub_velocity_status = Arc::new(
            node.create_publisher::<autoware_vehicle_msgs::msg::VelocityReport>(
                "vehicle/status/velocity_status".reliable(),
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

        // Create initial pose subscription
        let initial_pose = Arc::new(std::sync::Mutex::new(None));
        let initial_pose_cb = initial_pose.clone();
        let initialpose_sub = Arc::new(
            node.create_subscription::<geometry_msgs::msg::PoseWithCovarianceStamped, _>(
                "/initialpose".reliable().keep_last(1),
                move |msg: geometry_msgs::msg::PoseWithCovarianceStamped| {
                    tracing::info!(
                        "Initial pose received: ({:.2}, {:.2}, {:.2}) in frame '{}'",
                        msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z,
                        msg.header.frame_id
                    );

                    // Convert ROS pose to CARLA transform
                    let carla_isometry =
                        crate::coordinate_conversion::ros_pose_to_carla_isometry(&msg.pose.pose);
                    *initial_pose_cb.lock().unwrap() = Some(carla_isometry);
                },
            )?,
        );

        tracing::info!("Autoware coordinator initialized");

        Ok(Autoware {
            // === ROS Infrastructure ===
            node,

            // === Autoware Detection and Configuration ===
            detector,
            tf_buffer,
            sensor_configs: Vec::new(), // Will be populated by parse_sensors()

            // === Localization Publishers ===
            pub_localization,
            pub_tf,

            // === Vehicle Status Publishers ===
            pub_actuation_status,
            pub_velocity_status,
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
            _initialpose_sub: initialpose_sub,

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
                let lidar_key = format!("sensing/lidar/{sensor_name}/pointcloud");
                self.list_lidar.insert(sensor_name.clone(), lidar_key);
            }
            SensorType::LidarRayCastSemantic => {
                let lidar_key = format!("sensing/lidar/{sensor_name}/pointcloud");
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
    pub fn get_urdf(&self) -> Option<String> {
        self.detector.get_urdf()
    }

    /// Parse URDF and extract sensor configurations
    ///
    /// Parses the URDF from `/robot_description` and extracts sensor
    /// link information. Populates `sensor_configs` and registers
    /// sensor topics using `add_sensors()`.
    ///
    /// Must be called after Autoware is detected.
    ///
    /// # Returns
    /// Result indicating success or parsing error
    pub fn parse_sensors(&mut self) -> Result<()> {
        tracing::info!("Parsing URDF for sensor configurations...");

        let urdf = self.get_urdf().ok_or_else(|| {
            crate::error::BridgeError::AutowareIssue(
                "No URDF available - Autoware not detected".to_string(),
            )
        })?;

        // Parse URDF to extract sensor configurations
        self.sensor_configs = parse_urdf_sensors(&urdf)?;

        tracing::info!("Found {} sensors in URDF", self.sensor_configs.len());

        // Register sensor topics for each discovered sensor
        // Clone configs to avoid borrow checker issues
        let configs_to_register: Vec<_> = self
            .sensor_configs
            .iter()
            .map(|config| (config.sensor_type, config.link_name.clone()))
            .collect();

        for (sensor_type, link_name) in configs_to_register {
            tracing::debug!(
                "Registering sensor: {} (type: {:?})",
                link_name,
                sensor_type
            );
            self.add_sensors(sensor_type, link_name);
        }

        Ok(())
    }

    /// Get parsed sensor configurations
    ///
    /// Returns slice of sensor configurations extracted from URDF.
    /// Empty until `parse_sensors()` is called.
    ///
    /// # Returns
    /// Slice of SensorConfig structs
    pub fn sensor_configs(&self) -> &[SensorConfig] {
        &self.sensor_configs
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

    // === Initial Pose Methods ===

    /// Check if initial pose has been received
    ///
    /// Returns true if a pose message has been received on /initialpose.
    ///
    /// # Returns
    /// Boolean indicating if initial pose is available
    pub fn has_initial_pose(&self) -> bool {
        self.initial_pose.lock().unwrap().is_some()
    }

    /// Wait for initial pose to be received
    ///
    /// Blocks until the `/initialpose` topic receives a message.
    ///
    /// # Arguments
    /// * `timeout` - Optional timeout duration. None means wait forever.
    ///
    /// # Returns
    /// Result indicating success or timeout error
    pub fn wait_for_initial_pose(
        &self,
        timeout: Option<std::time::Duration>,
        running: &std::sync::Arc<std::sync::atomic::AtomicBool>,
        executor: &mut rclrs::Executor,
    ) -> Result<()> {
        tracing::info!("Waiting for initial pose from RViz...");
        tracing::info!("(Use '2D Pose Estimate' tool in RViz to set vehicle position)");

        let start = std::time::Instant::now();

        loop {
            // Spin executor to process ROS callbacks (e.g., /initialpose subscription)
            executor.spin(
                rclrs::SpinOptions::spin_once().timeout(std::time::Duration::from_millis(100)),
            );

            if self.has_initial_pose() {
                tracing::info!("Initial pose received!");
                return Ok(());
            }

            if !running.load(std::sync::atomic::Ordering::SeqCst) {
                tracing::info!("Shutdown requested while waiting for initial pose");
                return Err(crate::error::BridgeError::AutowareIssue(
                    "Shutdown requested".to_string(),
                ));
            }

            // Check timeout
            if let Some(timeout_duration) = timeout {
                if start.elapsed() >= timeout_duration {
                    return Err(crate::error::BridgeError::AutowareIssue(
                        "Timeout waiting for initial pose".to_string(),
                    ));
                }
            }
        }
    }

    /// Get the initial pose
    ///
    /// Returns a copy of the initial pose if available.
    ///
    /// # Returns
    /// Result containing the initial pose or error if not available
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
    pub fn node(&self) -> rclrs::Node {
        self.node.clone()
    }

    // === Vehicle Command Accessors ===

    /// Get current actuation command
    ///
    /// Returns the latest actuation command received from Autoware.
    ///
    /// # Returns
    /// Arc containing the latest ActuationCommandStamped
    pub fn get_actuation_cmd(&self) -> Arc<tier4_vehicle_msgs::msg::ActuationCommandStamped> {
        self.current_actuation_cmd.load_full()
    }

    /// Get current gear command
    ///
    /// Returns the latest gear command received from Autoware.
    ///
    /// # Returns
    /// Arc containing the latest GearCommand
    pub fn get_gear_cmd(&self) -> Arc<autoware_vehicle_msgs::msg::GearCommand> {
        self.current_gear_cmd.load_full()
    }

    /// Get current gate mode
    ///
    /// Returns the latest gate mode received from Autoware.
    ///
    /// # Returns
    /// Arc containing the latest GateMode
    pub fn get_gate_mode(&self) -> Arc<tier4_control_msgs::msg::GateMode> {
        self.current_gate_mode.load_full()
    }

    /// Get current turn indicators command
    ///
    /// Returns the latest turn indicators command received from Autoware.
    ///
    /// # Returns
    /// Arc containing the latest TurnIndicatorsCommand
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
    pub fn get_hazard_lights_cmd(&self) -> Arc<autoware_vehicle_msgs::msg::HazardLightsCommand> {
        self.current_hazard_lights_cmd.load_full()
    }

    // === Vehicle Status Publishing ===

    /// Publish vehicle localization (odometry and TF)
    ///
    /// Publishes the vehicle's pose, twist, and transform to Autoware's localization system.
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
    pub fn publish_localization(
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

        // Publish odometry
        self.pub_localization.publish(&odom)?;

        // Create and publish TF
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

        self.pub_tf.publish(&tf_msg)?;

        Ok(())
    }
}
