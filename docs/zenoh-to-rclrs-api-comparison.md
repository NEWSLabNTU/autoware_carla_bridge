# Zenoh to rclrs API Comparison

This document provides a comprehensive comparison between Zenoh and rclrs APIs, based on the analysis of the `zenoh_carla_bridge` codebase and the goal of translating it to use rclrs for ROS 2 native publishing.

## Table of Contents

- [Overview](#overview)
- [Zenoh API Usage in Current Code](#zenoh-api-usage-in-current-code)
- [Key API Components Comparison](#key-api-components-comparison)
  - [1. Initialization](#1-initialization)
  - [2. Publishers](#2-publishers)
  - [3. Subscribers](#3-subscribers)
  - [4. Quality of Service (QoS)](#4-quality-of-service-qos)
  - [5. Liveliness Tokens](#5-liveliness-tokens)
  - [6. Attachments](#6-attachments)
  - [7. Message Serialization](#7-message-serialization)
- [Migration Strategy](#migration-strategy)
- [Code Examples](#code-examples)
- [References](#references)

## Overview

**Zenoh** is a pub/sub, geo-distributed storage, and query protocol that provides flexible communication patterns. In the current `zenoh_carla_bridge`, Zenoh is used to bridge CARLA simulator data to ROS 2 systems through various bridge modes (DDS, ROS2, RmwZenoh).

**rclrs** is the official Rust client library for ROS 2, providing native ROS 2 node functionality without requiring an intermediate bridge.

## Zenoh API Usage in Current Code

Based on analysis of the codebase, here are the main Zenoh API patterns used:

### Files Using Zenoh API

1. **main.rs**: Session creation, configuration, and main loop
2. **sensor_bridge.rs**: Publisher declarations, data publishing with attachments
3. **vehicle_bridge.rs**: Publisher/Subscriber pairs for vehicle control
4. **autoware.rs**: Topic liveliness tokens and topic formatting
5. **clock.rs**: Clock publisher
6. **utils.rs**: Attachment generation macro

### Key Zenoh Concepts Used

1. **Session Management**: `zenoh::open()`, `Config`, `Arc<Session>`
2. **Publishers**: `declare_publisher()`, `put()`, `wait()`
3. **Subscribers**: `declare_subscriber()`, `callback_mut()`
4. **Liveliness Tokens**: `liveliness().declare_token()`, `undeclare()`
5. **Attachments**: Custom metadata for rmw_zenoh compatibility
6. **Key Expressions**: Topic name patterns

## Key API Components Comparison

### 1. Initialization

#### Zenoh (Current)

```rust
// Create config
let mut config = Config::default();
config.insert_json5("listen/endpoints", &json!(zenoh_listen).to_string())
    .expect("Failed to set zenoh listen endpoints.");

// Open session
let z_session = Arc::new(zenoh::open(config).wait()?);
```

**Location**: `main.rs:105-112`

#### rclrs (Target)

```rust
// Initialize context
let context = rclrs::Context::new(std::env::args())?;

// Create node
let node = rclrs::Node::new(context, "carla_bridge")?;
let node = Arc::new(node);
```

**Key Differences**:
- Zenoh uses a `Session` while rclrs uses `Context` + `Node`
- Zenoh config is flexible (JSON5), rclrs uses ROS 2 parameters
- rclrs requires a node name and namespace

### 2. Publishers

#### Zenoh (Current)

```rust
// Declare publisher
let publisher = z_session
    .declare_publisher(key.clone())
    .wait()?;

// Publish with attachment (rmw_zenoh mode)
put_with_attachment!(publisher, encoded, attachment, mode)?;

// Publish without attachment
publisher.put(payload).wait()?;
```

**Locations**:
- `sensor_bridge.rs:222,301,342,383,423`
- `vehicle_bridge.rs:96-116`

#### rclrs (Target)

```rust
// Create publisher with QoS
let publisher = node
    .create_publisher::<sensor_msgs::msg::Image>(
        "topic_name",
        rclrs::QOS_PROFILE_SENSOR_DATA
    )?;

// Publish
publisher.publish(&msg)?;
```

**Key Differences**:
- rclrs requires message type at compile time
- QoS is built into publisher creation
- No separate "wait()" needed
- No attachment mechanism (built into ROS 2 DDS layer)

### 3. Subscribers

#### Zenoh (Current)

```rust
// Declare subscriber with callback
let subscriber = z_session
    .declare_subscriber(autoware.topic_actuation_cmd.clone())
    .callback_mut(move |sample| {
        let result: Result<ActuationCommandStamped, _> =
            cdr::deserialize_from(sample.payload().reader(), cdr::size::Infinite);
        let Ok(cmd) = result else {
            log::error!("Unable to parse data");
            return;
        };
        // Process cmd
        cloned_cmd.store(Arc::new(cmd));
    })
    .wait()?;
```

**Location**: `vehicle_bridge.rs:132-143`

#### rclrs (Target)

```rust
// Create subscription with callback
let subscription = node
    .create_subscription::<ActuationCommandStamped, _>(
        "topic_name",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: ActuationCommandStamped| {
            // Process msg
            cloned_cmd.store(Arc::new(msg));
        }
    )?;
```

**Key Differences**:
- rclrs automatically deserializes messages
- No manual CDR deserialization needed
- QoS specified at creation time
- Message type safety at compile time

### 4. Quality of Service (QoS)

#### Zenoh (Current)

QoS information is encoded in topic metadata (for rmw_zenoh mode):

```rust
// QoS string format: "::,1:,:,:,,"
// Encoded in topic liveliness token
TopicInfo {
    entity_kind: "MP",
    type_name: "sensor_msgs::msg::dds_::Image_",
    type_hash: "TypeHashNotSupported",
    qos: "2::,5:,:,:,,"  // QoS string
}
```

**Location**: `autoware.rs:29-50`

#### rclrs (Target)

```rust
// Predefined QoS profiles
rclrs::QOS_PROFILE_DEFAULT
rclrs::QOS_PROFILE_SENSOR_DATA
rclrs::QOS_PROFILE_PARAMETERS

// Custom QoS
let qos = rclrs::QoSProfile {
    history: rclrs::QoSHistoryPolicy::KeepLast { depth: 10 },
    reliability: rclrs::QoSReliabilityPolicy::Reliable,
    durability: rclrs::QoSDurabilityPolicy::Volatile,
    ..Default::default()
};
```

**Key Differences**:
- rclrs uses structured QoS profiles
- Type-safe QoS configuration
- Standard ROS 2 QoS semantics

### 5. Liveliness Tokens

#### Zenoh (Current)

Liveliness tokens are used for ROS 2 graph discovery (rmw_zenoh mode):

```rust
// Declare node liveliness
fn declare_node_liveliness(&self, prefix: &str) {
    let zid = self.z_session.zid().to_string();
    let nid = CARLA_BRIDGE_NODE_ID;
    let node_name = NODE_NAME;
    let keyexpr = format!("{prefix}@ros2_lv/0/{zid}/{nid}/{nid}/NN/%/%/{node_name}");
    let token = self
        .z_session
        .liveliness()
        .declare_token(keyexpr)
        .wait()
        .unwrap();
    self.tokens.lock().unwrap().push(token);
}

// Declare topic liveliness
fn declare_topic_liveliness(&self, prefix: &str, base: &str, info: &TopicInfo) {
    let keyexpr = format!(
        "{}@ros2_lv/0/{}/{}/{}/{}/%/%/{}/{}/{}/{}",
        prefix, zid, nid, eid, info.entity_kind, node_name,
        mangled_qualified_name, info.type_name, info.type_hash, info.qos
    );
    let token = self.z_session.liveliness().declare_token(keyexpr).wait().unwrap();
    self.tokens.lock().unwrap().push(token);
}

// Cleanup
fn undeclare_all_liveliness(&self) {
    for t in tokens.drain(..) {
        t.undeclare().wait().ok();
    }
}
```

**Location**: `autoware.rs:77-149`

#### rclrs (Target)

```rust
// Not needed - ROS 2 handles discovery automatically
// The ROS 2 middleware (DDS/rmw) manages node and topic discovery
```

**Key Differences**:
- Liveliness tokens are a Zenoh-specific concept for ROS 2 graph compatibility
- rclrs doesn't need manual liveliness management
- ROS 2 discovery is handled by the middleware layer

### 6. Attachments

#### Zenoh (Current)

Attachments are used for rmw_zenoh compatibility:

```rust
// Generate rmw_zenoh-compatible attachment
pub fn generate_attachment() -> Vec<u8> {
    let seq_num: i64 = 1;
    let mut attachment = seq_num.to_le_bytes().to_vec();
    attachment.extend_from_slice(&0i64.to_le_bytes());
    attachment.push(16u8);
    attachment.extend_from_slice(&[0xAB; 16]);
    attachment
}

// Macro for conditional attachment
macro_rules! put_with_attachment {
    ($publisher:expr, $payload:expr, $attachment:expr, $mode:expr) => {
        if $mode == Mode::RmwZenoh {
            $publisher.put($payload).attachment($attachment.clone()).wait()
        } else {
            $publisher.put($payload).wait()
        }
    };
}
```

**Location**: `utils.rs:33-56`

#### rclrs (Target)

```rust
// Not needed - ROS 2 handles message metadata internally
```

**Key Differences**:
- Attachments are Zenoh-specific for rmw_zenoh compatibility
- rclrs uses standard ROS 2 message headers and metadata
- No manual attachment generation needed

### 7. Message Serialization

#### Zenoh (Current)

Manual CDR serialization:

```rust
use cdr::{CdrLe, Infinite};

// Serialize message
let encoded = cdr::serialize::<_, _, CdrLe>(&msg, Infinite)?;
publisher.put(encoded).wait()?;

// Deserialize message
let result: Result<ActuationCommandStamped, _> =
    cdr::deserialize_from(sample.payload().reader(), cdr::size::Infinite);
```

**Locations**:
- `sensor_bridge.rs:479,519,598,685,724,750`
- `vehicle_bridge.rs:233,268,287,300,318,332,351`

#### rclrs (Target)

```rust
// Automatic serialization/deserialization
publisher.publish(&msg)?;  // Automatically serializes

// Subscription callback receives deserialized message
node.create_subscription::<MsgType, _>(
    "topic",
    qos,
    |msg: MsgType| {
        // msg is already deserialized
    }
)?;
```

**Key Differences**:
- rclrs handles serialization automatically
- No manual CDR encoding/decoding
- Type safety guarantees correct serialization

## Migration Strategy

### Phase 1: Core Infrastructure

1. **Remove Zenoh dependencies** from `Cargo.toml`
2. **Add rclrs dependencies**:
   ```toml
   rclrs = "0.4"
   std_msgs = "*"
   sensor_msgs = "*"
   geometry_msgs = "*"
   # Autoware message types
   ```

3. **Replace Session with Context + Node**:
   - Remove `Arc<Session>` from all structs
   - Add `Arc<rclrs::Node>` instead
   - Initialize in `main.rs`

### Phase 2: Publisher Migration

1. **Update publisher declarations**:
   - Replace `declare_publisher()` with `create_publisher()`
   - Add message types and QoS profiles
   - Map Zenoh QoS strings to rclrs QoS profiles

2. **Remove attachment logic**:
   - Delete `utils::generate_attachment()`
   - Remove `put_with_attachment!` macro
   - Simplify publish calls

3. **Remove manual serialization**:
   - Delete CDR serialize calls
   - Publish messages directly

### Phase 3: Subscriber Migration

1. **Update subscriber declarations**:
   - Replace `declare_subscriber()` with `create_subscription()`
   - Remove manual deserialization from callbacks
   - Update callback signatures for typed messages

2. **Update data flow**:
   - Ensure Arc/Mutex patterns work with rclrs callbacks
   - Test thread safety

### Phase 4: Mode and Liveliness Removal

1. **Remove Mode enum**: The bridge will only support native ROS 2
2. **Remove liveliness token logic**: Not needed with rclrs
3. **Simplify topic naming**: Use standard ROS 2 topic names

### Phase 5: Testing and Validation

1. **Unit tests** for each bridge component
2. **Integration tests** with CARLA
3. **Verify topics** with `ros2 topic list/echo`
4. **Performance comparison** with Zenoh version

## Code Examples

### Example 1: Camera Publisher Migration

#### Before (Zenoh)

```rust
fn register_camera_rgb(
    z_session: Arc<Session>,
    actor: &Sensor,
    key_list: Option<Vec<String>>,
    tx: Sender<(MessageType, Vec<u8>)>,
    rx: Receiver<(MessageType, Vec<u8>)>,
    attachment: Vec<u8>,
    mode: Mode,
) -> Result<()> {
    let key_list = key_list.ok_or(BridgeError::CarlaIssue("No sensor exists"))?;
    let raw_key = key_list[0].clone();

    let image_publisher = z_session.declare_publisher(raw_key.clone()).wait()?;
    thread::spawn(move || loop {
        match rx.recv() {
            Ok((MessageType::SensorData, sensor_data)) => {
                if let Err(e) = put_with_attachment!(image_publisher, sensor_data, attachment, mode) {
                    log::error!("Failed to publish: {e:?}");
                }
            }
            _ => break,
        }
    });

    actor.listen(move |data| {
        let mut header = utils::create_ros_header(Some(data.timestamp()));
        header.frame_id = String::from("camera4/camera_link");
        if let Ok(data) = data.try_into() {
            if let Err(e) = camera_callback(header, data, &tx) {
                log::error!("Failed to call camera_callback: {e:?}");
            }
        }
    });
    Ok(())
}

fn camera_callback(
    header: std_msgs::Header,
    image: CarlaImage,
    tx: &Sender<(MessageType, Vec<u8>)>,
) -> Result<()> {
    let image_msg = sensor_msgs::Image {
        header,
        height: image.height() as u32,
        width: image.width() as u32,
        encoding: "bgra8".to_string(),
        is_bigendian: utils::is_bigendian().into(),
        step: (image.width() * 4) as u32,
        data: image.as_slice().iter().flat_map(|&Color { b, g, r, a }| [b, g, r, a]).collect(),
    };

    let encoded = cdr::serialize::<_, _, CdrLe>(&image_msg, Infinite)?;
    tx.send((MessageType::SensorData, encoded))?;
    Ok(())
}
```

#### After (rclrs)

```rust
fn register_camera_rgb(
    node: Arc<rclrs::Node>,
    actor: &Sensor,
    topic_name: String,
) -> Result<()> {
    let image_publisher = node.create_publisher::<sensor_msgs::msg::Image>(
        &topic_name,
        rclrs::QOS_PROFILE_SENSOR_DATA,
    )?;

    actor.listen(move |data| {
        let mut header = utils::create_ros_header(Some(data.timestamp()));
        header.frame_id = String::from("camera4/camera_link");

        if let Ok(carla_image) = data.try_into() {
            let image_msg = sensor_msgs::msg::Image {
                header,
                height: carla_image.height() as u32,
                width: carla_image.width() as u32,
                encoding: "bgra8".to_string(),
                is_bigendian: utils::is_bigendian() as u8,
                step: (carla_image.width() * 4) as u32,
                data: carla_image.as_slice()
                    .iter()
                    .flat_map(|&Color { b, g, r, a }| [b, g, r, a])
                    .collect(),
            };

            if let Err(e) = image_publisher.publish(&image_msg) {
                log::error!("Failed to publish image: {e:?}");
            }
        }
    });
    Ok(())
}
```

**Key Changes**:
- Removed channel-based threading (tx/rx)
- Removed manual CDR serialization
- Removed attachment handling
- Simplified to direct publish
- Type-safe message construction

### Example 2: Vehicle Subscriber Migration

#### Before (Zenoh)

```rust
let current_actuation_cmd = Arc::new(ArcSwap::from_pointee(ActuationCommandStamped {
    header: Header { stamp: Time { sec: 0, nanosec: 0 }, frame_id: "".to_string() },
    actuation: ActuationCommand { accel_cmd: 0.0, brake_cmd: 0.0, steer_cmd: 0.0 },
}));
let cloned_cmd = current_actuation_cmd.clone();

let subscriber_actuation_cmd = z_session
    .declare_subscriber(autoware.topic_actuation_cmd.clone())
    .callback_mut(move |sample| {
        let result: Result<ActuationCommandStamped, _> =
            cdr::deserialize_from(sample.payload().reader(), cdr::size::Infinite);
        let Ok(cmd) = result else {
            log::error!("Unable to parse data");
            return;
        };
        cloned_cmd.store(Arc::new(cmd));
    })
    .wait()?;
```

#### After (rclrs)

```rust
let current_actuation_cmd = Arc::new(ArcSwap::from_pointee(ActuationCommandStamped {
    header: Header { stamp: Time { sec: 0, nanosec: 0 }, frame_id: "".to_string() },
    actuation: ActuationCommand { accel_cmd: 0.0, brake_cmd: 0.0, steer_cmd: 0.0 },
}));
let cloned_cmd = current_actuation_cmd.clone();

let subscription_actuation_cmd = node
    .create_subscription::<ActuationCommandStamped, _>(
        &autoware.topic_actuation_cmd,
        rclrs::QOS_PROFILE_DEFAULT,
        move |cmd: ActuationCommandStamped| {
            cloned_cmd.store(Arc::new(cmd));
        }
    )?;
```

**Key Changes**:
- No manual deserialization needed
- Cleaner callback signature
- Automatic type checking
- Removed error-prone parsing

### Example 3: Main Loop and Spinning

#### Before (Zenoh)

```rust
// Main loop
while running.load(Ordering::SeqCst) {
    // Bridge management logic
    {
        // Add/remove actors
        // ...
    }

    // Update bridges
    let sec = world.snapshot().timestamp().elapsed_seconds;
    bridge_list.values_mut().try_for_each(|bridge| bridge.step(sec))?;

    world.wait_for_tick();
}
```

#### After (rclrs)

```rust
// Create executor for spinning
let executor = rclrs::SingleThreadedExecutor::new();

// Main loop
while running.load(Ordering::SeqCst) {
    // Bridge management logic
    {
        // Add/remove actors
        // ...
    }

    // Update bridges
    let sec = world.snapshot().timestamp().elapsed_seconds;
    bridge_list.values_mut().try_for_each(|bridge| bridge.step(sec))?;

    // Spin ROS callbacks
    executor.spin_once(Some(std::time::Duration::from_millis(10)))?;

    world.wait_for_tick();
}
```

**Key Changes**:
- Added ROS 2 executor for callback processing
- `spin_once()` processes ROS callbacks
- Timeout prevents blocking

## References

### Zenoh Resources

- **Zenoh API Docs**: https://docs.rs/zenoh/latest/zenoh/
- **Session**: https://docs.rs/zenoh/latest/zenoh/struct.Session.html
- **Publisher**: https://docs.rs/zenoh/latest/zenoh/pubsub/struct.Publisher.html
- **Subscriber**: https://docs.rs/zenoh/latest/zenoh/pubsub/struct.Subscriber.html
- **Liveliness**: https://docs.rs/zenoh/latest/zenoh/liveliness/index.html

### rclrs Resources

- **rclrs API Docs**: https://docs.rs/rclrs/latest/rclrs/
- **GitHub**: https://github.com/ros2-rust/ros2_rust
- **Tutorial**: https://github.com/ros2-rust/ros2_rust/blob/main/docs/writing-your-first-rclrs-node.md
- **QoS**: https://github.com/ros2-rust/ros2_rust/blob/main/rclrs/src/qos.rs

### ROS 2 Resources

- **ROS 2 Humble**: https://docs.ros.org/en/humble/
- **QoS Policies**: https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html
- **DDS Discovery**: https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html

---

**Document Version**: 1.0
**Last Updated**: 2025-10-20
**Author**: Claude Code (AI Assistant)
