use std::sync::{Arc, Mutex as StdMutex};

use r2r::builtin_interfaces::msg::Time;
use r2r::geometry_msgs::msg::{PointStamped, Quaternion, Twist, Vector3};
use r2r::nav_msgs::msg::Path;
use r2r::RosParams;
use tf_r2r::{TfError, TfListener};
use tokio::sync::Mutex;

use crate::Params;

use thiserror::Error;

#[derive(RosParams, Default, Debug)]
pub struct ControllerParams {
    // The distance to the carrot that we want to reach on the path
    carrot_distance: f32,
    // The maximum rotation velocity of the robot in rad/s around the z-axis
    max_rotation_vel: f64,
    // Clamped p gain of the rotation controller
    rotation_slow_down_factor: f64,
    // Clamped p gain of the translation controller
    translation_slow_down_factor: f64,
    // Distance at which we switch from orienting towards the path to orienting towards the goal poses orientation (in meters)
    orient_to_goal_distance: f64,
}

#[derive(Error, Debug)]
pub enum ControllerStepError {
    #[error("Could not determine the robot location")]
    LocalizationError(#[from] TfError),
}

pub struct Controller {
    tf: Arc<Mutex<TfListener>>,
    params: Arc<StdMutex<Params>>,
}

fn get_yaw_from_quaternion(quaternion: &Quaternion) -> f64 {
    glam::Quat::from_xyzw(
        quaternion.x as f32,
        quaternion.y as f32,
        quaternion.z as f32,
        quaternion.w as f32,
    )
    .to_euler(glam::EulerRot::ZYX)
    .0 as f64
}

impl Controller {
    pub fn new(tf: Arc<Mutex<TfListener>>, params: Arc<StdMutex<Params>>) -> Self {
        Self { tf, params }
    }

    // Calculates a command velocity based on a given path
    pub async fn step(&self, path: &Path) -> Result<(Twist, PointStamped), ControllerStepError> {
        let mut cmd_vel = Twist::default();

        let my_position = self
            .tf
            .lock()
            .await
            .lookup_transform(
                &self.params.lock().unwrap().map.frame,
                &self.params.lock().unwrap().base_footprint_frame,
                Time::default(),
            )
            .map_err(|e| ControllerStepError::LocalizationError(e))?
            .transform;

        let end_pose = &path
            .poses
            .last()
            .expect("Expect path to contain at least one pose")
            .pose;
        let goal_pose = {
            let carrot_distance = self.params.lock().unwrap().controller.carrot_distance as usize;
            if path.poses.len() > carrot_distance {
                &path.poses[path.poses.len() - carrot_distance].pose
            } else {
                end_pose
            }
        };

        let walk_angle = (goal_pose.position.y - my_position.translation.y)
            .atan2(goal_pose.position.x - my_position.translation.x);

        let distance = if path.poses.len() < 3 {
            (end_pose.position.x - my_position.translation.x)
                .hypot(end_pose.position.y - my_position.translation.y)
        } else {
            path.poses
                .iter()
                .fold(
                    (0.0, my_position.translation),
                    |(distance, last_pose), pose| {
                        let dx = pose.pose.position.x - last_pose.x;
                        let dy = pose.pose.position.y - last_pose.y;
                        (
                            distance + dx.hypot(dy),
                            Vector3 {
                                x: pose.pose.position.x,
                                y: pose.pose.position.y,
                                z: 0.0,
                            },
                        )
                    },
                )
                .0
        };

        let walk_vel = distance
            * self
                .params
                .lock()
                .unwrap()
                .controller
                .translation_slow_down_factor;

        let diff = if distance
            > self
                .params
                .lock()
                .unwrap()
                .controller
                .orient_to_goal_distance
        {
            walk_angle - get_yaw_from_quaternion(&my_position.rotation)
        } else {
            get_yaw_from_quaternion(&end_pose.orientation)
                - get_yaw_from_quaternion(&my_position.rotation)
        };

        let min_angle = diff % std::f64::consts::TAU; // TODO check math / remainder implementation

        cmd_vel.angular.z = (min_angle
            * self
                .params
                .lock()
                .unwrap()
                .controller
                .rotation_slow_down_factor)
            .clamp(
                -self.params.lock().unwrap().controller.max_rotation_vel,
                self.params.lock().unwrap().controller.max_rotation_vel,
            );

        let local_heading = walk_angle - get_yaw_from_quaternion(&my_position.rotation);
        let local_heading_vector_x = local_heading.cos();
        let local_heading_vector_y = local_heading.sin();

        let carrot_point = PointStamped {
            header: path.header.clone(),
            point: goal_pose.position.clone(),
        };

        Ok((cmd_vel, carrot_point))
    }
}
