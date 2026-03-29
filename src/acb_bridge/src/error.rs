pub type Result<T, E = BridgeError> = std::result::Result<T, E>;

#[derive(Debug, thiserror::Error)]
pub enum BridgeError {
    #[error("ROS 2 error: {0}")]
    Rclrs(#[from] rclrs::RclrsError),

    /// Communication error - reserved for future network/communication failure handling
    #[error("Communication error: {0}")]
    #[allow(dead_code)]
    Communication(&'static str),

    /// Error variant for sensors without owners - reserved for future error handling
    #[error("The sensor with ID {sensor_id} is ownerless")]
    #[allow(dead_code)]
    OwnerlessSensor { sensor_id: u32 },

    #[error("The issue is from Carla: {0}")]
    CarlaIssue(&'static str),

    #[error("CARLA error: {0}")]
    Carla(#[from] carla::CarlaError),

    #[error("Autoware detection error: {0}")]
    AutowareIssue(String),

    #[error("Configuration error: {0}")]
    ConfigError(String),

    #[error("{0}")]
    Other(#[from] Box<dyn std::error::Error + Sync + Send + 'static>),
}
