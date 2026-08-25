use std::{convert::Infallible, mem, str::FromStr, sync::Arc};

use bytemuck::{Pod, Zeroable};
use carla::{
    client::{ActorBase, Sensor},
    // `SensorDataBase` gives `SensorData::timestamp()`, the simulation time the measurement
    // was taken. That is mapped onto Autoware's `/clock` epoch by `utils::SimClockOffset`;
    // reading the node clock inside the callback instead stamps the tick, not the sample.
    sensor::SensorDataBase,
    sensor::data::{
        Color, GnssMeasurement, Image as CarlaImage, ImuMeasurement, LidarMeasurement,
        SemanticLidarMeasurement,
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

/// Autoware 2025 PointXYZIRC format (16 bytes per point)
///
/// This struct matches Autoware's expected point cloud format for NDT localization.
/// Fields: x, y, z (float32), intensity (uint8), return_type (uint8), channel (uint16)
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
#[repr(C, packed)]
struct PointXYZIRC {
    x: f32,
    y: f32,
    z: f32,
    intensity: u8,
    return_type: u8,
    channel: u16,
}

impl PointXYZIRC {
    const POINT_STEP: u32 = 16;

    /// Create PointCloud2 field definitions for this point format
    fn point_fields() -> Vec<sensor_msgs::msg::PointField> {
        vec![
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
                datatype: PointFieldType::UINT8 as u8,
                count: 1,
            },
            sensor_msgs::msg::PointField {
                name: "return_type".to_string(),
                offset: 13,
                datatype: PointFieldType::UINT8 as u8,
                count: 1,
            },
            sensor_msgs::msg::PointField {
                name: "channel".to_string(),
                offset: 14,
                datatype: PointFieldType::UINT16 as u8,
                count: 1,
            },
        ]
    }
}

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
    /// Vehicle name - stored for potential future use in logging/debugging
    _vehicle_name: String,
    /// Sensor type - stored for potential future sensor-specific processing
    _sensor_type: SensorType,
    /// CARLA sensor actor - must be kept alive to maintain sensor listeners
    _actor: Sensor,
    /// Sensor name - stored for potential future use in logging/debugging
    _sensor_name: String,
}

impl SensorBridge {
    pub fn get_bridge_type(actor: Sensor) -> Result<BridgeType> {
        let sensor_id = actor.id();
        let sensor_type_id = actor.type_id();

        // Check if sensor has a parent (should have one if attached to vehicle)
        let _parent = actor
            .parent()?
            .ok_or(BridgeError::OwnerlessSensor { sensor_id })?;

        // Get sensor name
        let sensor_name = actor
            .attributes()?
            .iter()
            .find(|attr| attr.id() == "role_name")
            .map(|attr| attr.value_string())
            .unwrap_or_else(|| generate_sensor_name(&actor));

        // Parse sensor type
        let sensor_type: SensorType = sensor_type_id.parse().or(Err(BridgeError::CarlaIssue(
            "Unable to recognize sensor type",
        )))?;

        tracing::info!("Detected sensor '{sensor_name}' (type: {:?})", sensor_type);
        Ok(BridgeType::Sensor(sensor_type, sensor_name))
    }

    pub fn new(
        node: rclrs::Node,
        actor: Sensor,
        bridge_type: BridgeType,
        autoware: &Autoware,
        clock_offset: utils::SimClockOffset,
    ) -> Result<SensorBridge> {
        let (sensor_type, sensor_name) = match bridge_type {
            BridgeType::Sensor(t, s) => (t, s),
            _ => panic!("SensorBridge::new called with non-Sensor bridge type!"),
        };

        let key_list = autoware.get_sensors_key(sensor_type, &sensor_name);

        let node = Arc::new(node);

        match sensor_type {
            SensorType::CameraRgb => {
                register_camera_rgb(node.clone(), &actor, key_list, &sensor_name)?;
            }
            SensorType::LidarRayCast => {
                register_lidar_raycast(node.clone(), &actor, key_list, &sensor_name)?;
            }
            SensorType::LidarRayCastSemantic => {
                register_lidar_raycast_semantic(node.clone(), &actor, key_list, &sensor_name)?;
            }
            SensorType::Imu => {
                register_imu(node.clone(), &actor, key_list, &sensor_name, clock_offset)?;
            }
            SensorType::Gnss => {
                register_gnss(node.clone(), &actor, key_list, &sensor_name)?;
            }
            SensorType::Collision => {
                tracing::warn!("Collision sensor is not supported yet");
            }
            SensorType::NotSupport => {
                tracing::warn!("Unsupported sensor type '{}'", actor.type_id());
            }
        }

        Ok(SensorBridge {
            _vehicle_name: String::new(), // No vehicle name in 1-to-1 mode
            _sensor_type: sensor_type,
            _actor: actor,
            _sensor_name: sensor_name,
        })
    }
}

impl ActorBridge for SensorBridge {
    fn step(&mut self, _timestamp: f64) -> Result<()> {
        Ok(())
    }
}

impl Drop for SensorBridge {
    fn drop(&mut self) {
        tracing::info!(
            "Dropping sensor bridge: {} (type: {:?}) - sensor owned by CarlaVehicle",
            self._sensor_name,
            self._sensor_type
        );
        // NOTE: We don't destroy the sensor here because CarlaVehicle owns the sensors
        // and is responsible for destroying them in cleanup(). The sensor reference
        // in SensorBridge keeps callbacks alive while the bridge exists.
    }
}

fn register_camera_rgb(
    node: Arc<rclrs::Node>,
    actor: &Sensor,
    key_list: Option<Vec<String>>,
    frame_id: &str,
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
        .attributes()?
        .iter()
        .find(|attr| attr.id() == "image_size_x")
        .ok_or(BridgeError::CarlaIssue("no image_size_x"))?
        .value()
        .ok_or(BridgeError::CarlaIssue("no such ActorAttributeValueKind"))?
        .try_into_int()
        .or(Err(BridgeError::CarlaIssue("Unable to transform into int")))? as u32;
    let height = actor
        .attributes()?
        .iter()
        .find(|attr| attr.id() == "image_size_y")
        .ok_or(BridgeError::CarlaIssue("no image_size_y"))?
        .value()
        .ok_or(BridgeError::CarlaIssue("no such ActorAttributeValueKind"))?
        .try_into_int()
        .or(Err(BridgeError::CarlaIssue("Unable to transform into int")))? as u32;
    let fov = actor
        .attributes()?
        .iter()
        .find(|attr| attr.id() == "fov")
        .ok_or(BridgeError::CarlaIssue("no fov"))?
        .value()
        .ok_or(BridgeError::CarlaIssue("no such ActorAttributeValueKind"))?
        .try_into_f32()
        .or(Err(BridgeError::CarlaIssue("Unable to transform into f32")))? as f64;

    // Images and camera_info are stamped with the *optical* frame, not the mounting
    // frame. Everything that projects a 3D point into this image -- Autoware's traffic
    // light map-based detector among them -- assumes the REP-103 optical convention of
    // z forward, x right, y down, and reads that frame off the header. Stamped with
    // `<ns>/camera_link` (x forward, z up) instead, the detector's forward ray comes out
    // pointing at the sky, every projected point lands behind the image plane, and it
    // silently emits no regions of interest at all.
    let frame_id = match frame_id.strip_suffix("/camera_link") {
        Some(namespace) => format!("{namespace}/camera_optical_link"),
        None => frame_id.to_string(),
    };
    let clock_node = node.clone();

    // Setup CARLA listener
    actor.listen(move |data| {
        let mut header = utils::create_ros_header_from_node(&clock_node);
        header.frame_id = frame_id.clone();

        if let Ok(carla_image) = data.try_into() {
            // Publish image
            if let Err(e) = publish_camera_image(&image_publisher, header.clone(), carla_image) {
                tracing::error!("Failed to publish camera image: {e:?}");
            }

            // Publish camera info
            if let Err(e) = publish_camera_info(&info_publisher, header, width, height, fov) {
                tracing::error!("Failed to publish camera info: {e:?}");
            }
        } else {
            tracing::error!("Failed to transform camera image");
        }
    })?;

    Ok(())
}

fn register_lidar_raycast(
    node: Arc<rclrs::Node>,
    actor: &Sensor,
    key_list: Option<Vec<String>>,
    frame_id: &str,
) -> Result<()> {
    let key_list = key_list.ok_or(BridgeError::CarlaIssue("No sensor exists"))?;
    let topic = key_list[0].clone();

    let publisher = Arc::new(
        node.create_publisher::<sensor_msgs::msg::PointCloud2>(topic.as_str().sensor_data_qos())?,
    );

    // Clone frame_id for closure
    let frame_id = frame_id.to_string();
    let clock_node = node.clone();

    actor.listen(move |data| {
        let mut header = utils::create_ros_header_from_node(&clock_node);
        header.frame_id = frame_id.clone();

        if let Ok(measure) = data.try_into() {
            if let Err(e) = publish_lidar(&publisher, header, measure) {
                tracing::error!("Failed to publish lidar data: {e:?}");
            }
        } else {
            tracing::error!("Failed to transform lidar data");
        }
    })?;

    Ok(())
}

fn register_lidar_raycast_semantic(
    node: Arc<rclrs::Node>,
    actor: &Sensor,
    key_list: Option<Vec<String>>,
    frame_id: &str,
) -> Result<()> {
    let key_list = key_list.ok_or(BridgeError::CarlaIssue("No sensor exists"))?;
    let topic = key_list[0].clone();

    let publisher = Arc::new(
        node.create_publisher::<sensor_msgs::msg::PointCloud2>(topic.as_str().sensor_data_qos())?,
    );

    // Clone frame_id for closure
    let frame_id = frame_id.to_string();
    let clock_node = node.clone();

    actor.listen(move |data| {
        let mut header = utils::create_ros_header_from_node(&clock_node);
        header.frame_id = frame_id.clone();

        if let Ok(measure) = data.try_into() {
            if let Err(e) = publish_semantic_lidar(&publisher, header, measure) {
                tracing::error!("Failed to publish semantic lidar data: {e:?}");
            }
        } else {
            tracing::error!("Failed to transform semantic lidar data");
        }
    })?;

    Ok(())
}

fn register_imu(
    node: Arc<rclrs::Node>,
    actor: &Sensor,
    key_list: Option<Vec<String>>,
    frame_id: &str,
    clock_offset: utils::SimClockOffset,
) -> Result<()> {
    let key_list = key_list.ok_or(BridgeError::CarlaIssue("No sensor exists"))?;
    let topic = key_list[0].clone();

    let publisher = Arc::new(node.create_publisher::<sensor_msgs::msg::Imu>(topic.as_str())?);

    // Clone frame_id for closure
    let frame_id = frame_id.to_string();
    let clock_node = node.clone();

    actor.listen(move |data| {
        // Stamp with the simulation time the measurement was taken, mapped onto `/clock`.
        // Falling back to the node clock covers the first callbacks, before the main loop
        // has seen a frame to anchor the offset against.
        let carla_stamp = data.timestamp();
        let mut header = match clock_offset.stamp(carla_stamp) {
            Some(stamp) => std_msgs::msg::Header {
                stamp,
                frame_id: String::new(),
            },
            None => utils::create_ros_header_from_node(&clock_node),
        };
        header.frame_id = frame_id.clone();

        if let Ok(measure) = data.try_into() {
            if let Err(e) = publish_imu(&publisher, header, measure) {
                tracing::error!("Failed to publish IMU data: {e:?}");
            }
        } else {
            tracing::error!("Failed to transform IMU data");
        }
    })?;

    Ok(())
}

fn register_gnss(
    node: Arc<rclrs::Node>,
    actor: &Sensor,
    key_list: Option<Vec<String>>,
    frame_id: &str,
) -> Result<()> {
    let key_list = key_list.ok_or(BridgeError::CarlaIssue("No sensor exists"))?;
    let topic = key_list[0].clone();

    let publisher = Arc::new(node.create_publisher::<sensor_msgs::msg::NavSatFix>(topic.as_str())?);

    // Clone frame_id for closure
    let frame_id = frame_id.to_string();
    let clock_node = node.clone();

    actor.listen(move |data| {
        let mut header = utils::create_ros_header_from_node(&clock_node);
        header.frame_id = frame_id.clone();

        if let Ok(measure) = data.try_into() {
            if let Err(e) = publish_gnss(&publisher, header, measure) {
                tracing::error!("Failed to publish GNSS data: {e:?}");
            }
        } else {
            tracing::error!("Failed to transform GNSS data");
        }
    })?;

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

    // Get channel information from CARLA's LidarMeasurement
    let channel_count = measure.channel_count();

    // Build channel boundaries: cumulative point counts per channel
    // This allows us to determine which channel each point belongs to
    let mut channel_boundaries: Vec<usize> = Vec::with_capacity(channel_count + 1);
    channel_boundaries.push(0);
    for ch in 0..channel_count {
        let prev = *channel_boundaries.last().unwrap();
        let count = measure.point_count(ch).unwrap_or(0);
        channel_boundaries.push(prev + count);
    }

    // Convert CARLA points to Autoware PointXYZIRC format using declarative struct
    //
    // The boundary table is sorted and CARLA emits points in channel order, so the
    // channel for each point is found by advancing a cursor -- one comparison for all
    // but `channel_count` points per scan. The old code linear-scanned the whole table
    // per point: ~8.4 M comparisons per scan at 128 channels. See issue 011.
    let mut cursor = 0usize;
    let points: Vec<PointXYZIRC> = lidar_data
        .iter()
        .enumerate()
        .map(|(idx, det)| {
            while cursor + 1 < channel_boundaries.len() && idx >= channel_boundaries[cursor + 1] {
                cursor += 1;
            }
            // CARLA does not promise the ordering; fall back to a binary search if the
            // cursor ran past this point's channel.
            let channel = if idx >= channel_boundaries[cursor] {
                cursor
            } else {
                channel_boundaries
                    .partition_point(|&b| b <= idx)
                    .saturating_sub(1)
            } as u16;

            PointXYZIRC {
                // Apply Y-axis flip: CARLA uses left-handed (Y=right), ROS uses right-handed (Y=left)
                // The pre-built PCD map is in ROS frame (Y-flipped), so live scan must match.
                x: det.point.x,
                y: -det.point.y,
                z: det.point.z,
                // Convert float intensity [0.0-1.0] to uint8 [0-255]
                intensity: (det.intensity.clamp(0.0, 1.0) * 255.0) as u8,
                return_type: 0, // Single return
                channel,
            }
        })
        .collect();

    // Use bytemuck for zero-copy serialization
    let data: Vec<u8> = bytemuck::cast_slice(&points).to_vec();
    let row_step = data.len() as u32;

    let lidar_msg = sensor_msgs::msg::PointCloud2 {
        header,
        height: 1,
        width: points.len() as u32,
        fields: PointXYZIRC::point_fields(),
        is_bigendian: utils::is_bigendian(),
        point_step: PointXYZIRC::POINT_STEP,
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
            // Use CARLA coordinates directly (no Y-axis flip)
            // The pointcloud map was created with CARLA's native coordinate system
            let (x, y, z) = (det.point.x, det.point.y, det.point.z);
            let cos_inc_angle = det.cos_inc_angle;
            let object_idx = det.object_idx;
            let object_tag = det.object_tag;
            [
                x.to_ne_bytes(),
                y.to_ne_bytes(),
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

/// Convert CARLA's compass heading to a ROS yaw in the map frame.
///
/// CARLA's IMU reports `compass` in radians measured **clockwise from north**. The map
/// frame this bridge publishes into has `x` = CARLA `x` (east) and `y` = `-CARLA_y`
/// (north), so ROS yaw is measured counter-clockwise from east:
///
/// ```text
/// ros_yaw = pi/2 - compass
/// ```
///
/// It used to be `compass.atan2(-compass)`, which is `3pi/4` for every positive reading
/// and `-pi/4` for every negative one -- a constant. See issue 001.
fn compass_to_ros_yaw(compass: f32) -> f32 {
    std::f32::consts::FRAC_PI_2 - compass
}

fn publish_imu(
    publisher: &Arc<rclrs::Publisher<sensor_msgs::msg::Imu>>,
    header: std_msgs::msg::Header,
    measure: ImuMeasurement,
) -> Result<()> {
    let accel = measure.accelerometer();
    let gyro = measure.gyroscope();
    let compass = measure.compass();

    let quat = UnitQuaternion::from_euler_angles(0.0, 0.0, compass_to_ros_yaw(compass));
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
        // Angular velocity is a pseudovector, so the CARLA -> ROS Y-flip does NOT act on
        // it the way it acts on the accelerometer below. See issue 002.
        angular_velocity: geometry_msgs::msg::Vector3 {
            x: -gyro.x as f64,
            y: gyro.y as f64,
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
        // Negate latitude: CARLA maps CARLA_y (right=south) to Northing without sign flip,
        // but the TUMFTM maps use y = -CARLA_y convention (y = ROS left = map north).
        // Negating latitude makes gnss_poser output match the map frame (y = -CARLA_y).
        latitude: -measure.latitude(),
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::{FRAC_PI_2, PI};

    /// Regression guard for issue 001. The old `compass.atan2(-compass)` returned
    /// `3*pi/4` for every positive reading, so all three of these collapsed to one value.
    #[test]
    fn compass_maps_each_heading_to_a_distinct_ros_yaw() {
        // Facing east: CARLA yaw 0, compass pi/2, ROS yaw 0 (+x is east).
        assert!((compass_to_ros_yaw(FRAC_PI_2) - 0.0).abs() < 1e-6);
        // Facing north: CARLA yaw -90 deg, compass 0, ROS yaw +pi/2 (+y is north).
        assert!((compass_to_ros_yaw(0.0) - FRAC_PI_2).abs() < 1e-6);
        // Facing west: compass 3*pi/2, ROS yaw -pi (equivalently +pi).
        assert!((compass_to_ros_yaw(3.0 * FRAC_PI_2) + PI).abs() < 1e-6);
    }

    /// The ROS yaw must turn the opposite way to the compass: a compass reading that
    /// increases (turning clockwise/east) is a ROS yaw that decreases.
    #[test]
    fn compass_and_ros_yaw_turn_in_opposite_directions() {
        let a = compass_to_ros_yaw(0.1);
        let b = compass_to_ros_yaw(0.2);
        assert!(b < a);
    }

    /// Regression guard for issue 002. Angular velocity is a pseudovector: under the
    /// CARLA -> ROS reflection `diag(1,-1,1)` it maps as `(-x, y, -z)`, unlike the
    /// accelerometer's `(x, -y, z)`.
    #[test]
    fn gyro_and_accel_use_different_sign_rules() {
        let gyro = (1.0_f64, 2.0_f64, 3.0_f64);
        let accel = (1.0_f64, 2.0_f64, 3.0_f64);

        let gyro_ros = (-gyro.0, gyro.1, -gyro.2);
        let accel_ros = (accel.0, -accel.1, accel.2);

        assert_eq!(gyro_ros, (-1.0, 2.0, -3.0));
        assert_eq!(accel_ros, (1.0, -2.0, 3.0));
        assert_ne!(gyro_ros.0.signum(), accel_ros.0.signum());
        assert_ne!(gyro_ros.1.signum(), accel_ros.1.signum());
    }

    /// The cursor walk in `publish_lidar` must land on the same channel a linear scan
    /// would, including the fall back for out-of-order points. See issue 011.
    #[test]
    fn channel_lookup_matches_a_linear_scan() {
        let boundaries = [0usize, 3, 3, 7, 10]; // note the empty channel 1
        let linear = |idx: usize| -> usize {
            boundaries
                .windows(2)
                .position(|w| idx >= w[0] && idx < w[1])
                .unwrap_or(0)
        };

        let mut cursor = 0usize;
        for idx in 0..10 {
            while cursor + 1 < boundaries.len() && idx >= boundaries[cursor + 1] {
                cursor += 1;
            }
            let walked = if idx >= boundaries[cursor] {
                cursor
            } else {
                boundaries.partition_point(|&b| b <= idx).saturating_sub(1)
            };
            assert_eq!(walked, linear(idx), "point {idx}");
        }
    }
}
