use futures::prelude::*;
use glam::{EulerRot, Quat};
use r2r::{
    geometry_msgs::msg::{Quaternion, Transform, TransformStamped, Vector3},
    std_msgs::msg::Header,
    tf2_msgs::msg::TFMessage,
    Clock,
    RosParams,
};
use std::sync::{Arc, Mutex as StdMutex};

#[derive(RosParams, Debug, Clone)]
struct CalibrationParams {
    /// Parent frame for the calibration transform
    parent_frame: String,
    /// Child frame for the calibration transform
    child_frame: String,
    /// Roll offset in radians
    offset_x: f32,
    /// Pitch offset in radians
    offset_y: f32,
    /// Yaw offset in radians
    offset_z: f32,
}

impl Default for CalibrationParams {
    fn default() -> Self {
        Self {
            parent_frame: "camera_optical_frame_uncalibrated".to_string(),
            child_frame: "camera_optical_frame".to_string(),
            offset_x: 0.,
            offset_y: 0.,
            offset_z: 0.,
        }
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "bitbots_extrinsic_calibration", "")?;
    let nl = node.logger().to_string();

    // Initialize parameter handling
    let params = Arc::new(StdMutex::new(CalibrationParams::default()));
    let (parameter_handler, mut parameter_events) =
        node.make_derived_parameter_handler(params.clone())?;
    tokio::task::spawn(parameter_handler);

    let tf_static_broadcaster = node.create_publisher("/tf_static", r2r::QosProfile::default())?;
    let ros_clock = node.get_ros_clock();

    tokio::task::spawn(async move {
        loop {
            parameter_events.next().await;
            let params = params.lock().unwrap();

            r2r::log_info!(&nl, "New parameters: {:#?}", params);

            tf_static_broadcaster
                .publish(&TFMessage {
                    transforms: vec![TransformStamped {
                        header: Header {
                            frame_id: params.parent_frame.clone(),
                            stamp: Clock::to_builtin_time(&ros_clock.lock().unwrap().get_now().unwrap()),
                        },
                        child_frame_id: params.child_frame.clone(),
                        transform: Transform {
                            translation: Vector3::default(),
                            rotation: {
                                let quat = Quat::from_euler(
                                    EulerRot::ZYX,
                                    params.offset_z,
                                    params.offset_y,
                                    params.offset_x,
                                );
                                Quaternion {
                                    x: quat.x as f64,
                                    y: quat.y as f64,
                                    z: quat.z as f64,
                                    w: quat.w as f64,
                                }
                            },
                        },
                    }],
                })
                .expect("Failed to publish updated transform");
        }
    });

    // Spin the node
    loop {
        node.spin_once(std::time::Duration::from_millis(100));
    }
}
