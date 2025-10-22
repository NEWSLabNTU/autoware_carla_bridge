pub type Result<T, E = BridgeError> = std::result::Result<T, E>;

#[derive(Debug, thiserror::Error)]
pub enum BridgeError {
    #[error("ROS 2 error: {0}")]
    Rclrs(#[from] rclrs::RclrsError),

    /// Communication error - reserved for future network/communication failure handling
    #[error("Communication error: {0}")]
    #[allow(dead_code)]
    Communication(&'static str),

    #[error("The sensor with ID {sensor_id} is ownerless")]
    OwnerlessSensor { sensor_id: u32 },

    #[error("The vehicle is NPC")]
    Npc { npc_role_name: String },

    #[error("The issue is from Carla: {0}")]
    CarlaIssue(&'static str),

    #[error("{0}")]
    Other(#[from] Box<dyn std::error::Error + Sync + Send + 'static>),
}
