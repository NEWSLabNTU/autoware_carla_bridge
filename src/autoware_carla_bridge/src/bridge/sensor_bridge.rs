use std::{convert::Infallible, mem, str::FromStr, sync::Arc};

use carla::{
    client::{ActorBase, Sensor},
    geom::Location,
    sensor::{
        data::{
            Color, GnssMeasurement, Image as CarlaImage, ImuMeasurement, LidarDetection,
            LidarMeasurement, SemanticLidarDetection, SemanticLidarMeasurement,
        },
        SensorDataBase,
    },
};
use nalgebra::{coordinates::XYZ, UnitQuaternion};
use rclrs::IntoPrimitiveOptions;

use super::actor_bridge::{ActorBridge, BridgeType};
use crate::{
    autoware::Autoware,
    error::{BridgeError, Result},
    types::{GnssService, GnssStatus, PointFieldType},
    utils,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SensorType {
    CameraRgb,
    LidarRayCast,
    LidarRayCastSemantic,
    Imu,
    Gnss,
    Collision,
    NotSupport,
}

impl FromStr for SensorType {
    type Err = Infallible;

    fn from_str(type_id: &str) -> Result<Self, Self::Err> {
        Ok(match type_id {
            "sensor.camera.rgb" => SensorType::CameraRgb,
            "sensor.lidar.ray_cast" => SensorType::LidarRayCast,
            "sensor.lidar.ray_cast_semantic" => SensorType::LidarRayCastSemantic,
            "sensor.other.imu" => SensorType::Imu,
            "sensor.other.gnss" => SensorType::Gnss,
            "sensor.other.collision" => SensorType::Collision,
            _ => SensorType::NotSupport,
        })
    }
}

pub struct SensorBridge {
    _vehicle_name: String,
    sensor_type: SensorType,
    _actor: Sensor,
    sensor_name: String,
}

impl SensorBridge {
    pub fn get_bridge_type(actor: Sensor) -> Result<BridgeType> {
        let sensor_id = actor.id();
        let sensor_type_id = actor.type_id();

        let mut vehicle_name = actor
            .parent()
            .ok_or(BridgeError::OwnerlessSensor { sensor_id })?
            .attributes()
            .iter()
            .find(|attr| attr.id() == "role_name")
            .ok_or(BridgeError::CarlaIssue("'role_name' attribute is missing"))?
            .value_string();
        let sensor_name = actor
            .attributes()
            .iter()
            .find(|attr| attr.id() == "role_name")
            .map(|attr| attr.value_string())
            .unwrap_or_else(|| generate_sensor_name(&actor));

        // Remove "autoware_" in role name
        if !vehicle_name.starts_with("autoware_") {
            return Err(BridgeError::Npc {
                npc_role_name: vehicle_name,
            });
        }
        vehicle_name = vehicle_name.replace("autoware_", "");

        log::info!("Detected a sensor '{sensor_name}' on '{vehicle_name}'");
        let sensor_type: SensorType = sensor_type_id.parse().or(Err(BridgeError::CarlaIssue(
            "Unable to recognize sensor type",
        )))?;
        Ok(BridgeType::Sensor(vehicle_name, sensor_type, sensor_name))
    }

    pub fn new(
        node: rclrs::Node,
        actor: Sensor,
        bridge_type: BridgeType,
        autoware: &Autoware,
    ) -> Result<SensorBridge> {
        let (vehicle_name, sensor_type, sensor_name) = match bridge_type {
            BridgeType::Sensor(v, t, s) => (v, t, s),
            _ => panic!("Should never happen!"),
        };

        let key_list = autoware.get_sensors_key(sensor_type, &sensor_name);

        let node = Arc::new(node);

        match sensor_type {
            SensorType::CameraRgb => {
                register_camera_rgb(node.clone(), &actor, key_list)?;
            }
            SensorType::LidarRayCast => {
                register_lidar_raycast(node.clone(), &actor, key_list)?;
            }
            SensorType::LidarRayCastSemantic => {
                register_lidar_raycast_semantic(node.clone(), &actor, key_list)?;
            }
            SensorType::Imu => {
                register_imu(node.clone(), &actor, key_list)?;
            }
            SensorType::Gnss => {
                register_gnss(node.clone(), &actor, key_list)?;
            }
            SensorType::Collision => {
                log::warn!("Collision sensor is not supported yet");
            }
            SensorType::NotSupport => {
                log::warn!("Unsupported sensor type '{}'", actor.type_id());
            }
        }

        Ok(SensorBridge {
            _vehicle_name: vehicle_name,
            sensor_type,
            _actor: actor,
            sensor_name,
        })
    }
}

impl ActorBridge for SensorBridge {
    fn step(&mut self, _timestamp: f64) -> Result<()> {
        Ok(())
    }
}

fn register_camera_rgb(
    node: Arc<rclrs::Node>,
    actor: &Sensor,
    key_list: Option<Vec<String>>,
) -> Result<()> {
    let key_list = key_list.ok_or(BridgeError::CarlaIssue("No sensor exists"))?;
    let raw_topic = key_list[0].clone();
    let info_topic = key_list[1].clone();

    // Create publishers
    let image_publisher = Arc::new(
        node.create_publisher::<sensor_msgs::msg::Image>(raw_topic.as_str().sensor_data_qos())?,
    );
    let info_publisher =
        Arc::new(node.create_publisher::<sensor_msgs::msg::CameraInfo>(
            info_topic.as_str().sensor_data_qos(),
        )?);

    // Get camera parameters
    let width = actor
        .attributes()
        .iter()
        .find(|attr| attr.id() == "image_size_x")
        .ok_or(BridgeError::CarlaIssue("no image_size_x"))?
        .value()
        .ok_or(BridgeError::CarlaIssue("no such ActorAttributeValueKind"))?
        .try_into_int()
        .or(Err(BridgeError::CarlaIssue("Unable to transform into int")))? as u32;
    let height = actor
        .attributes()
        .iter()
        .find(|attr| attr.id() == "image_size_y")
        .ok_or(BridgeError::CarlaIssue("no image_size_y"))?
        .value()
        .ok_or(BridgeError::CarlaIssue("no such ActorAttributeValueKind"))?
        .try_into_int()
        .or(Err(BridgeError::CarlaIssue("Unable to transform into int")))? as u32;
    let fov = actor
        .attributes()
        .iter()
        .find(|attr| attr.id() == "fov")
        .ok_or(BridgeError::CarlaIssue("no fov"))?
        .value()
        .ok_or(BridgeError::CarlaIssue("no such ActorAttributeValueKind"))?
        .try_into_f32()
        .or(Err(BridgeError::CarlaIssue("Unable to transform into f32")))? as f64;

    // Setup CARLA listener
    actor.listen(move |data| {
        let mut header = utils::create_ros_header(Some(data.timestamp()));
        header.frame_id = String::from("camera4/camera_link");

        if let Ok(carla_image) = data.try_into() {
            // Publish image
            if let Err(e) = publish_camera_image(&image_publisher, header.clone(), carla_image) {
                log::error!("Failed to publish camera image: {e:?}");
            }

            // Publish camera info
            if let Err(e) = publish_camera_info(&info_publisher, header, width, height, fov) {
                log::error!("Failed to publish camera info: {e:?}");
            }
        } else {
            log::error!("Failed to transform camera image");
        }
    });

    Ok(())
}

fn register_lidar_raycast(
    node: Arc<rclrs::Node>,
    actor: &Sensor,
    key_list: Option<Vec<String>>,
) -> Result<()> {
    let key_list = key_list.ok_or(BridgeError::CarlaIssue("No sensor exists"))?;
    let topic = key_list[0].clone();

    let publisher = Arc::new(
        node.create_publisher::<sensor_msgs::msg::PointCloud2>(topic.as_str().sensor_data_qos())?,
    );

    actor.listen(move |data| {
        let mut header = utils::create_ros_header(Some(data.timestamp()));
        header.frame_id = String::from("velodyne_top_base_link");

        if let Ok(measure) = data.try_into() {
            if let Err(e) = publish_lidar(&publisher, header, measure) {
                log::error!("Failed to publish lidar data: {e:?}");
            }
        } else {
            log::error!("Failed to transform lidar data");
        }
    });

    Ok(())
}

fn register_lidar_raycast_semantic(
    node: Arc<rclrs::Node>,
    actor: &Sensor,
    key_list: Option<Vec<String>>,
) -> Result<()> {
    let key_list = key_list.ok_or(BridgeError::CarlaIssue("No sensor exists"))?;
    let topic = key_list[0].clone();

    let publisher = Arc::new(
        node.create_publisher::<sensor_msgs::msg::PointCloud2>(topic.as_str().sensor_data_qos())?,
    );

    actor.listen(move |data| {
        let mut header = utils::create_ros_header(Some(data.timestamp()));
        header.frame_id = String::from("velodyne_top_base_link");

        if let Ok(measure) = data.try_into() {
            if let Err(e) = publish_semantic_lidar(&publisher, header, measure) {
                log::error!("Failed to publish semantic lidar data: {e:?}");
            }
        } else {
            log::error!("Failed to transform semantic lidar data");
        }
    });

    Ok(())
}

fn register_imu(
    node: Arc<rclrs::Node>,
    actor: &Sensor,
    key_list: Option<Vec<String>>,
) -> Result<()> {
    let key_list = key_list.ok_or(BridgeError::CarlaIssue("No sensor exists"))?;
    let topic = key_list[0].clone();

    let publisher = Arc::new(node.create_publisher::<sensor_msgs::msg::Imu>(topic.as_str())?);

    actor.listen(move |data| {
        let mut header = utils::create_ros_header(Some(data.timestamp()));
        header.frame_id = String::from("tamagawa/imu_link");

        if let Ok(measure) = data.try_into() {
            if let Err(e) = publish_imu(&publisher, header, measure) {
                log::error!("Failed to publish IMU data: {e:?}");
            }
        } else {
            log::error!("Failed to transform IMU data");
        }
    });

    Ok(())
}

fn register_gnss(
    node: Arc<rclrs::Node>,
    actor: &Sensor,
    key_list: Option<Vec<String>>,
) -> Result<()> {
    let key_list = key_list.ok_or(BridgeError::CarlaIssue("No sensor exists"))?;
    let topic = key_list[0].clone();

    let publisher = Arc::new(node.create_publisher::<sensor_msgs::msg::NavSatFix>(topic.as_str())?);

    actor.listen(move |data| {
        let mut header = utils::create_ros_header(Some(data.timestamp()));
        header.frame_id = String::from("gnss_link");

        if let Ok(measure) = data.try_into() {
            if let Err(e) = publish_gnss(&publisher, header, measure) {
                log::error!("Failed to publish GNSS data: {e:?}");
            }
        } else {
            log::error!("Failed to transform GNSS data");
        }
    });

    Ok(())
}

// Helper functions to build and publish messages

fn publish_camera_image(
    publisher: &Arc<rclrs::Publisher<sensor_msgs::msg::Image>>,
    header: std_msgs::msg::Header,
    image: CarlaImage,
) -> Result<()> {
    let image_data = image.as_slice();
    if image_data.is_empty() {
        return Ok(());
    }

    let width = image.width();
    let height = image.height();
    let data: Vec<_> = image_data
        .iter()
        .flat_map(|&Color { b, g, r, a }| [b, g, r, a])
        .collect();

    let image_msg = sensor_msgs::msg::Image {
        header,
        height: height as u32,
        width: width as u32,
        encoding: "bgra8".to_string(),
        is_bigendian: utils::is_bigendian() as u8,
        step: (width * 4) as u32,
        data,
    };

    publisher.publish(&image_msg)?;
    Ok(())
}

fn publish_camera_info(
    publisher: &Arc<rclrs::Publisher<sensor_msgs::msg::CameraInfo>>,
    header: std_msgs::msg::Header,
    width: u32,
    height: u32,
    fov: f64,
) -> Result<()> {
    let cx = width as f64 / 2.0;
    let cy = height as f64 / 2.0;
    let fx = width as f64 / (2.0 * (fov * std::f64::consts::PI / 360.0).tan());
    let fy = fx;

    let camera_info = sensor_msgs::msg::CameraInfo {
        header,
        width,
        height,
        distortion_model: String::from("plumb_bob"),
        k: [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        d: vec![0.0, 0.0, 0.0, 0.0, 0.0],
        r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p: [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        binning_x: 0,
        binning_y: 0,
        roi: sensor_msgs::msg::RegionOfInterest {
            x_offset: 0,
            y_offset: 0,
            height: 0,
            width: 0,
            do_rectify: false,
        },
    };

    publisher.publish(&camera_info)?;
    Ok(())
}

fn publish_lidar(
    publisher: &Arc<rclrs::Publisher<sensor_msgs::msg::PointCloud2>>,
    header: std_msgs::msg::Header,
    measure: LidarMeasurement,
) -> Result<()> {
    let lidar_data = measure.as_slice();
    if lidar_data.is_empty() {
        return Ok(());
    }

    let point_step = mem::size_of_val(&lidar_data[0]) as u32;
    let data: Vec<_> = lidar_data
        .iter()
        .flat_map(|det| {
            let LidarDetection {
                point: Location { x, y, z },
                intensity,
            } = *det;
            [y, x, z, intensity]
        })
        .flat_map(|elem| elem.to_ne_bytes())
        .collect();
    let row_step = data.len() as u32;

    let fields = vec![
        sensor_msgs::msg::PointField {
            name: "x".to_string(),
            offset: 0,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::msg::PointField {
            name: "y".to_string(),
            offset: 4,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::msg::PointField {
            name: "z".to_string(),
            offset: 8,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::msg::PointField {
            name: "intensity".to_string(),
            offset: 12,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
    ];

    let lidar_msg = sensor_msgs::msg::PointCloud2 {
        header,
        height: 1,
        width: lidar_data.len() as u32,
        fields,
        is_bigendian: utils::is_bigendian(),
        point_step,
        row_step,
        data,
        is_dense: true,
    };

    publisher.publish(&lidar_msg)?;
    Ok(())
}

fn publish_semantic_lidar(
    publisher: &Arc<rclrs::Publisher<sensor_msgs::msg::PointCloud2>>,
    header: std_msgs::msg::Header,
    measure: SemanticLidarMeasurement,
) -> Result<()> {
    let lidar_data = measure.as_slice();
    if lidar_data.is_empty() {
        return Ok(());
    }

    let point_step = mem::size_of_val(&lidar_data[0]) as u32;
    let data: Vec<_> = lidar_data
        .iter()
        .flat_map(|det| {
            let SemanticLidarDetection {
                point: Location { x, y, z },
                cos_inc_angle,
                object_idx,
                object_tag,
            } = *det;
            [
                y.to_ne_bytes(),
                x.to_ne_bytes(),
                z.to_ne_bytes(),
                cos_inc_angle.to_ne_bytes(),
                object_idx.to_ne_bytes(),
                object_tag.to_ne_bytes(),
            ]
        })
        .flatten()
        .collect();
    let row_step = data.len() as u32;

    let fields = vec![
        sensor_msgs::msg::PointField {
            name: "x".to_string(),
            offset: 0,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::msg::PointField {
            name: "y".to_string(),
            offset: 4,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::msg::PointField {
            name: "z".to_string(),
            offset: 8,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::msg::PointField {
            name: "CosAngle".to_string(),
            offset: 12,
            datatype: PointFieldType::FLOAT32 as u8,
            count: 1,
        },
        sensor_msgs::msg::PointField {
            name: "ObjIdx".to_string(),
            offset: 16,
            datatype: PointFieldType::UINT32 as u8,
            count: 1,
        },
        sensor_msgs::msg::PointField {
            name: "ObjTag".to_string(),
            offset: 20,
            datatype: PointFieldType::UINT32 as u8,
            count: 1,
        },
    ];

    let lidar_msg = sensor_msgs::msg::PointCloud2 {
        header,
        height: 1,
        width: lidar_data.len() as u32,
        fields,
        is_bigendian: utils::is_bigendian(),
        point_step,
        row_step,
        data,
        is_dense: true,
    };

    publisher.publish(&lidar_msg)?;
    Ok(())
}

fn publish_imu(
    publisher: &Arc<rclrs::Publisher<sensor_msgs::msg::Imu>>,
    header: std_msgs::msg::Header,
    measure: ImuMeasurement,
) -> Result<()> {
    let accel = measure.accelerometer();
    let gyro = measure.gyroscope();
    let compass = measure.compass();

    // Convert compass (north vector) to quaternion orientation
    let yaw = compass.atan2(-compass);
    let quat = UnitQuaternion::from_euler_angles(0.0, 0.0, yaw);
    let XYZ {
        x: qx,
        y: qy,
        z: qz,
    } = *quat.vector();
    let qw = quat.scalar();

    let imu_msg = sensor_msgs::msg::Imu {
        header,
        orientation: geometry_msgs::msg::Quaternion {
            x: qx as f64,
            y: qy as f64,
            z: qz as f64,
            w: qw as f64,
        },
        orientation_covariance: [0.0; 9],
        angular_velocity: geometry_msgs::msg::Vector3 {
            x: gyro.x as f64,
            y: -gyro.y as f64,
            z: -gyro.z as f64,
        },
        angular_velocity_covariance: [0.0; 9],
        linear_acceleration: geometry_msgs::msg::Vector3 {
            x: accel.x as f64,
            y: -accel.y as f64,
            z: accel.z as f64,
        },
        linear_acceleration_covariance: [0.0; 9],
    };

    publisher.publish(&imu_msg)?;
    Ok(())
}

fn publish_gnss(
    publisher: &Arc<rclrs::Publisher<sensor_msgs::msg::NavSatFix>>,
    header: std_msgs::msg::Header,
    measure: GnssMeasurement,
) -> Result<()> {
    let gnss_msg = sensor_msgs::msg::NavSatFix {
        header,
        status: sensor_msgs::msg::NavSatStatus {
            status: GnssStatus::GbasFix as i8,
            service: GnssService::Gps as u16,
        },
        latitude: measure.latitude(),
        longitude: measure.longitude(),
        altitude: measure.attitude(),
        position_covariance: [0.0; 9],
        position_covariance_type: 0,
    };

    publisher.publish(&gnss_msg)?;
    Ok(())
}

fn generate_sensor_name(actor: &Sensor) -> String {
    let id = actor.id();
    let type_id = actor.type_id();
    format!("{type_id}_{id}")
}
