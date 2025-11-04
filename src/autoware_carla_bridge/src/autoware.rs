use std::collections::HashMap;

use crate::bridge::sensor_bridge::SensorType;

#[derive(Clone)]
pub struct Autoware {
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
    pub fn new() -> Autoware {
        Autoware {
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
        }
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
}
