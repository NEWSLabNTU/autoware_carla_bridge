use crate::error::{BridgeError, Result};
use geometry_msgs::msg::TransformStamped;
use rclrs::IntoPrimitiveOptions;
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};

/// TF Buffer for storing and looking up coordinate transformations
///
/// Subscribes to /tf_static and maintains a local buffer of transforms
/// between frames. Supports multi-level transform composition.
pub struct TFBuffer {
    /// Map from child_frame_id to TransformStamped
    transforms: Arc<Mutex<HashMap<String, TransformStamped>>>,
    /// ROS subscription handle (kept alive)
    _tf_static_sub: Arc<rclrs::Subscription<tf2_msgs::msg::TFMessage>>,
}

impl TFBuffer {
    /// Create a new TF buffer and subscribe to /tf_static
    ///
    /// # Arguments
    /// * `node` - ROS node handle
    ///
    /// # Returns
    /// TFBuffer instance with active /tf_static subscription
    pub fn new(node: rclrs::Node) -> Result<Self> {
        let transforms = Arc::new(Mutex::new(HashMap::new()));
        let transforms_cb = transforms.clone();

        // Subscribe to /tf_static with TRANSIENT_LOCAL QoS
        let tf_static_sub = node
            .create_subscription::<tf2_msgs::msg::TFMessage, _>(
                "/tf_static".reliable().transient_local(),
                move |msg: tf2_msgs::msg::TFMessage| {
                    let mut tf_map = transforms_cb.lock().unwrap();

                    for transform in &msg.transforms {
                        tracing::debug!(
                            "TF: {} → {} [{:.3}, {:.3}, {:.3}]",
                            transform.header.frame_id,
                            transform.child_frame_id,
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z
                        );

                        tf_map.insert(transform.child_frame_id.clone(), transform.clone());
                    }

                    tracing::info!("TF Buffer updated: {} transforms", tf_map.len());
                },
            )
            .map_err(|e| {
                BridgeError::AutowareIssue(format!("Failed to subscribe to /tf_static: {}", e))
            })?;

        tracing::info!("TFBuffer initialized, subscribing to /tf_static");

        Ok(Self {
            transforms,
            _tf_static_sub: Arc::new(tf_static_sub),
        })
    }

    /// Look up a transform from source frame to target frame
    ///
    /// # Arguments
    /// * `target_frame` - Target frame ID (e.g., "base_link")
    /// * `source_frame` - Source frame ID (e.g., "velodyne_top")
    ///
    /// # Returns
    /// Transform from source to target frame
    pub fn lookup_transform(
        &self,
        target_frame: &str,
        source_frame: &str,
    ) -> Result<TransformStamped> {
        // If source and target are the same, return identity
        if source_frame == target_frame {
            return Ok(self.identity_transform(source_frame, target_frame));
        }

        let tf_map = self.transforms.lock().unwrap();

        // Try direct lookup (source → target)
        if let Some(tf) = tf_map.get(source_frame) {
            if tf.header.frame_id == target_frame {
                return Ok(tf.clone());
            }
        }

        // Try reverse lookup (target → source) and invert
        if let Some(tf) = tf_map.get(target_frame) {
            if tf.header.frame_id == source_frame {
                return Ok(self.invert_transform(tf));
            }
        }

        // Walk up from source to target through the TF tree
        let mut current_frame = source_frame.to_string();
        let mut chain: Vec<TransformStamped> = Vec::new();
        let max_depth = 20; // Prevent infinite loops

        for _ in 0..max_depth {
            if current_frame == target_frame {
                // Reached target - compose all transforms in the chain
                return self.compose_transform_chain(&chain, target_frame, source_frame);
            }

            // Get transform for current frame (parent → current)
            if let Some(tf) = tf_map.get(&current_frame) {
                let parent = tf.header.frame_id.clone();
                chain.push(tf.clone());
                current_frame = parent;
            } else {
                break; // No parent found
            }
        }

        // For complex hierarchies, we'd need to implement graph traversal
        // For now, just report the error
        Err(BridgeError::AutowareIssue(format!(
            "Transform from '{}' to '{}' not found in TF buffer (tried walking {} frames)",
            source_frame,
            target_frame,
            chain.len()
        )))
    }

    /// Compose a chain of transforms into a single transform
    fn compose_transform_chain(
        &self,
        chain: &[TransformStamped],
        _target_frame: &str,
        source_frame: &str,
    ) -> Result<TransformStamped> {
        if chain.is_empty() {
            return Err(BridgeError::AutowareIssue(
                "Empty transform chain".to_string(),
            ));
        }

        // Start with the first transform
        let mut result = chain[0].clone();

        // Compose each subsequent transform
        for tf in &chain[1..] {
            result = self.multiply_transforms(&result, tf)?;
        }

        // Update the child_frame_id to be the source
        result.child_frame_id = source_frame.to_string();

        Ok(result)
    }

    /// Multiply two transforms (compose them)
    fn multiply_transforms(
        &self,
        t1: &TransformStamped,
        t2: &TransformStamped,
    ) -> Result<TransformStamped> {
        // Convert to nalgebra for easier composition
        let trans1 = &t1.transform.translation;
        let rot1 = &t1.transform.rotation;
        let trans2 = &t2.transform.translation;
        let rot2 = &t2.transform.rotation;

        // Create nalgebra types
        let iso1 = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(trans1.x, trans1.y, trans1.z),
            nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
                rot1.w, rot1.x, rot1.y, rot1.z,
            )),
        );

        let iso2 = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(trans2.x, trans2.y, trans2.z),
            nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
                rot2.w, rot2.x, rot2.y, rot2.z,
            )),
        );

        // Compose: result = t2 * t1 (apply t1 first, then t2)
        let result_iso = iso2 * iso1;

        // Convert back to ROS message
        let mut result = t2.clone();
        result.transform.translation.x = result_iso.translation.x;
        result.transform.translation.y = result_iso.translation.y;
        result.transform.translation.z = result_iso.translation.z;
        result.transform.rotation.w = result_iso.rotation.w;
        result.transform.rotation.x = result_iso.rotation.i;
        result.transform.rotation.y = result_iso.rotation.j;
        result.transform.rotation.z = result_iso.rotation.k;
        result.header.frame_id = t2.header.frame_id.clone();
        result.child_frame_id = t1.child_frame_id.clone();

        Ok(result)
    }

    /// Get transform for a specific frame (relative to its parent)
    ///
    /// # Arguments
    /// * `frame_id` - Frame ID to look up
    ///
    /// # Returns
    /// Transform for the frame (parent → frame)
    ///
    /// NOTE: Public API method kept for direct transform lookups. Use `lookup_transform()`
    /// for chain traversal.
    #[allow(dead_code)]
    pub fn get_transform(&self, frame_id: &str) -> Result<TransformStamped> {
        let tf_map = self.transforms.lock().unwrap();

        tf_map.get(frame_id).cloned().ok_or_else(|| {
            BridgeError::AutowareIssue(format!("Frame '{}' not found in TF buffer", frame_id))
        })
    }

    /// Get all available frame IDs
    pub fn get_all_frames(&self) -> Vec<String> {
        let tf_map = self.transforms.lock().unwrap();
        tf_map.keys().cloned().collect()
    }

    /// Get number of transforms in buffer
    ///
    /// NOTE: Public API method kept for diagnostic and monitoring purposes.
    #[allow(dead_code)]
    pub fn len(&self) -> usize {
        let tf_map = self.transforms.lock().unwrap();
        tf_map.len()
    }

    /// Check if buffer is empty
    ///
    /// NOTE: Public API method kept for diagnostic and monitoring purposes.
    #[allow(dead_code)]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Create an identity transform
    fn identity_transform(&self, parent: &str, child: &str) -> TransformStamped {
        TransformStamped {
            header: std_msgs::msg::Header {
                frame_id: parent.to_string(),
                ..Default::default()
            },
            child_frame_id: child.to_string(),
            transform: geometry_msgs::msg::Transform {
                translation: geometry_msgs::msg::Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                rotation: geometry_msgs::msg::Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        }
    }

    /// Invert a transform (swap parent/child and invert translation/rotation)
    fn invert_transform(&self, tf: &TransformStamped) -> TransformStamped {
        // For a proper implementation, we'd need to:
        // 1. Invert the quaternion rotation
        // 2. Rotate the translation by the inverted rotation and negate
        // For now, this is a placeholder

        TransformStamped {
            header: std_msgs::msg::Header {
                frame_id: tf.child_frame_id.clone(),
                ..Default::default()
            },
            child_frame_id: tf.header.frame_id.clone(),
            transform: geometry_msgs::msg::Transform {
                translation: geometry_msgs::msg::Vector3 {
                    x: -tf.transform.translation.x,
                    y: -tf.transform.translation.y,
                    z: -tf.transform.translation.z,
                },
                rotation: geometry_msgs::msg::Quaternion {
                    x: -tf.transform.rotation.x,
                    y: -tf.transform.rotation.y,
                    z: -tf.transform.rotation.z,
                    w: tf.transform.rotation.w,
                },
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity_transform() {
        // Create a mock TFBuffer without actual ROS node
        // For real testing, we'd need integration tests with ROS
    }
}
