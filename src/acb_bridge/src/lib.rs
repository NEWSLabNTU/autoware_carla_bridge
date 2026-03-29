pub mod autoware;
pub mod autoware_detection;
pub mod bridge;
pub mod carla_vehicle;
pub mod clock;
pub mod coordinate_conversion;
pub mod error;
pub mod sensor_config;
pub mod tf_bridge;
pub mod types;
pub mod urdf_parser;
pub mod utils;

// Re-export commonly used items
pub use autoware_detection::{AutowareDetector, AutowareState, DetectionDiagnostics};
pub use bridge::sensor_bridge::SensorType;
pub use carla_vehicle::CarlaVehicle;
pub use error::{BridgeError, Result};
pub use sensor_config::{SensorType as ConfigSensorType, VehicleConfig};
pub use tf_bridge::TFBuffer;
