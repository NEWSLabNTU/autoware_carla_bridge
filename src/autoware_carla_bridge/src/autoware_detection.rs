use crate::error::{BridgeError, Result};
use rclrs::IntoPrimitiveOptions;
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    time::{Duration, Instant},
};

/// Detection state for Autoware instance
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutowareState {
    /// Autoware not detected yet
    NotDetected,
    /// Autoware instance detected and alive
    Detected,
    /// Autoware was detected but is now lost
    Lost,
}

/// Autoware instance detector
///
/// Detects Autoware instances via `/robot_description` topic and monitors
/// their lifecycle. Handles the transition between NotDetected → Detected → Lost states.
pub struct AutowareDetector {
    /// ROS node handle
    /// NOTE: Kept for future use in health checks and node queries
    #[allow(dead_code)]
    node: rclrs::Node,

    /// Subscription to /robot_description topic
    /// NOTE: This field is never accessed directly but must be kept alive to receive callbacks.
    /// The subscription is cancelled when this struct is dropped.
    #[allow(dead_code)]
    robot_desc_sub: Arc<rclrs::Subscription<std_msgs::msg::String>>,

    /// Flag indicating if robot_description has been received
    robot_desc_received: Arc<AtomicBool>,

    /// Latest URDF content from /robot_description
    latest_urdf: Arc<Mutex<Option<String>>>,

    /// Current detection state
    state: Arc<Mutex<AutowareState>>,

    /// Last time robot_description was received (for health check)
    last_heartbeat: Arc<Mutex<Option<Instant>>>,

    /// Detection timeout duration
    /// NOTE: Kept for wait_for_detection() method (currently unused but part of public API)
    #[allow(dead_code)]
    detection_timeout: Duration,

    /// Health check timeout duration (for detecting loss)
    health_timeout: Duration,
}

impl AutowareDetector {
    /// Create a new AutowareDetector
    ///
    /// # Arguments
    ///
    /// * `node` - ROS node handle
    /// * `detection_timeout` - How long to wait for initial detection (default: 60s)
    /// * `health_timeout` - How long without updates before marking as lost (default: 5s)
    ///
    /// # Returns
    ///
    /// AutowareDetector instance with /robot_description subscription set up
    pub fn new(
        node: rclrs::Node,
        detection_timeout: Option<Duration>,
        health_timeout: Option<Duration>,
    ) -> Result<Self> {
        let detection_timeout = detection_timeout.unwrap_or(Duration::from_secs(60));
        let health_timeout = health_timeout.unwrap_or(Duration::from_secs(5));

        let robot_desc_received = Arc::new(AtomicBool::new(false));
        let latest_urdf = Arc::new(Mutex::new(None));
        let state = Arc::new(Mutex::new(AutowareState::NotDetected));
        let last_heartbeat = Arc::new(Mutex::new(None));

        // Clone for callback
        let robot_desc_received_cb = robot_desc_received.clone();
        let latest_urdf_cb = latest_urdf.clone();
        let state_cb = state.clone();
        let last_heartbeat_cb = last_heartbeat.clone();

        // Subscribe to /robot_description with TRANSIENT_LOCAL QoS
        // This ensures we get the latched message even if published before subscription
        let robot_desc_sub = node
            .create_subscription::<std_msgs::msg::String, _>(
                "/robot_description".reliable().transient_local(),
                move |msg: std_msgs::msg::String| {
                    let urdf_size = msg.data.len();

                    // Store URDF
                    *latest_urdf_cb.lock().unwrap() = Some(msg.data.clone());

                    // Update state
                    let mut current_state = state_cb.lock().unwrap();
                    let was_detected = *current_state == AutowareState::Detected;

                    if !was_detected {
                        tracing::info!(
                            "Autoware detected: robot_description received ({} bytes)",
                            urdf_size
                        );
                        *current_state = AutowareState::Detected;
                    }

                    // Mark as received and update heartbeat
                    robot_desc_received_cb.store(true, Ordering::SeqCst);
                    *last_heartbeat_cb.lock().unwrap() = Some(Instant::now());
                },
            )
            .map_err(|e| {
                BridgeError::AutowareIssue(format!(
                    "Failed to subscribe to /robot_description: {}",
                    e
                ))
            })?;

        tracing::info!(
            "AutowareDetector initialized (detection_timeout: {:?}, health_timeout: {:?})",
            detection_timeout,
            health_timeout
        );

        Ok(Self {
            node,
            robot_desc_sub: Arc::new(robot_desc_sub),
            robot_desc_received,
            latest_urdf,
            state,
            last_heartbeat,
            detection_timeout,
            health_timeout,
        })
    }

    /// Get current detection state
    pub fn state(&self) -> AutowareState {
        *self.state.lock().unwrap()
    }

    /// Check if Autoware is currently detected and alive
    pub fn is_alive(&self) -> bool {
        self.state() == AutowareState::Detected
    }

    /// Get the latest URDF if available
    pub fn get_urdf(&self) -> Option<String> {
        self.latest_urdf.lock().unwrap().clone()
    }

    /// Wait for Autoware detection with timeout
    ///
    /// Blocks until Autoware is detected or timeout expires.
    ///
    /// # Returns
    ///
    /// Ok(()) if detected, Err if timeout
    ///
    /// NOTE: Alternative to is_alive() check, kept for explicit detection workflows
    #[allow(dead_code)]
    pub fn wait_for_detection(&self) -> Result<()> {
        let start = Instant::now();

        tracing::info!("Waiting for Autoware detection...");

        while !self.robot_desc_received.load(Ordering::SeqCst) {
            if start.elapsed() > self.detection_timeout {
                return Err(BridgeError::AutowareIssue(format!(
                    "Autoware detection timeout after {:?}",
                    self.detection_timeout
                )));
            }

            // Short sleep to avoid busy waiting
            std::thread::sleep(Duration::from_millis(100));
        }

        tracing::info!("Autoware detected successfully");
        Ok(())
    }

    /// Check for /robot_state_publisher node presence
    ///
    /// This is an additional check to verify Autoware components are running
    ///
    /// NOTE: Diagnostic utility kept for health checking
    #[allow(dead_code)]
    pub fn check_robot_state_publisher(&self) -> Result<bool> {
        let node_names = self
            .node
            .get_node_names()
            .map_err(|e| BridgeError::AutowareIssue(format!("Failed to get node names: {}", e)))?;

        let found = node_names
            .iter()
            .any(|name| name.name.contains("robot_state_publisher"));

        if found {
            tracing::debug!("robot_state_publisher node found");
        } else {
            tracing::debug!("robot_state_publisher node not found");
        }

        Ok(found)
    }

    /// Check for /tf_static topic presence
    ///
    /// This verifies that TF static transforms are being published
    ///
    /// NOTE: Diagnostic utility kept for health checking
    #[allow(dead_code)]
    pub fn check_tf_static_topic(&self) -> Result<bool> {
        let topics = self
            .node
            .get_topic_names_and_types()
            .map_err(|e| BridgeError::AutowareIssue(format!("Failed to get topics: {}", e)))?;

        let found = topics.iter().any(|(name, _)| name == "/tf_static");

        if found {
            tracing::debug!("/tf_static topic found");
        } else {
            tracing::debug!("/tf_static topic not found");
        }

        Ok(found)
    }

    /// Perform health check to detect if Autoware has disappeared
    ///
    /// Should be called periodically. Transitions to Lost state if no
    /// updates received within health_timeout.
    pub fn health_check(&self) {
        let mut current_state = self.state.lock().unwrap();

        match *current_state {
            AutowareState::Detected => {
                // Check if we've received updates recently
                let last_update = self.last_heartbeat.lock().unwrap();

                if let Some(last_time) = *last_update {
                    if last_time.elapsed() > self.health_timeout {
                        tracing::warn!(
                            "Autoware health check failed: no updates for {:?}",
                            last_time.elapsed()
                        );
                        tracing::warn!("Autoware marked as Lost");
                        *current_state = AutowareState::Lost;
                    }
                }
            }
            AutowareState::Lost => {
                // Check if Autoware has come back
                let received = self.robot_desc_received.load(Ordering::SeqCst);
                let last_update = self.last_heartbeat.lock().unwrap();

                if received {
                    if let Some(last_time) = *last_update {
                        if last_time.elapsed() < self.health_timeout {
                            tracing::info!("Autoware has recovered");
                            *current_state = AutowareState::Detected;
                        }
                    }
                }
            }
            AutowareState::NotDetected => {
                // Check if we've been detected
                if self.robot_desc_received.load(Ordering::SeqCst) {
                    tracing::info!("Autoware detected during health check");
                    *current_state = AutowareState::Detected;
                }
            }
        }
    }

    /// Get diagnostic information about detection state
    ///
    /// NOTE: Diagnostic API kept for monitoring and debugging purposes
    #[allow(dead_code)]
    pub fn get_diagnostics(&self) -> DetectionDiagnostics {
        let state = self.state();
        let urdf_available = self.latest_urdf.lock().unwrap().is_some();
        let last_update = self.last_heartbeat.lock().unwrap().map(|t| t.elapsed());

        DetectionDiagnostics {
            state,
            urdf_available,
            urdf_size: self.latest_urdf.lock().unwrap().as_ref().map(|u| u.len()),
            time_since_last_update: last_update,
        }
    }
}

/// Diagnostic information about Autoware detection
///
/// NOTE: Diagnostic struct kept for monitoring and debugging purposes
#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct DetectionDiagnostics {
    pub state: AutowareState,
    pub urdf_available: bool,
    pub urdf_size: Option<usize>,
    pub time_since_last_update: Option<Duration>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_state_transitions() {
        // Test state enum equality
        assert_eq!(AutowareState::NotDetected, AutowareState::NotDetected);
        assert_ne!(AutowareState::NotDetected, AutowareState::Detected);
        assert_ne!(AutowareState::Detected, AutowareState::Lost);
    }

    #[test]
    fn test_detection_diagnostics() {
        let diag = DetectionDiagnostics {
            state: AutowareState::Detected,
            urdf_available: true,
            urdf_size: Some(1024),
            time_since_last_update: Some(Duration::from_secs(1)),
        };

        assert_eq!(diag.state, AutowareState::Detected);
        assert!(diag.urdf_available);
        assert_eq!(diag.urdf_size, Some(1024));
    }
}
