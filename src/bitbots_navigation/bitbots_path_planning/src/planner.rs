use std::sync::Arc;
use std::sync::Mutex as StdMutex;

use crate::Params;
use bitbots_rust_nav::map::ObstacleMap;
use bitbots_rust_nav::map::ObstacleMapConfig;
use bitbots_rust_nav::obstacle::RoundObstacle;
use r2r::builtin_interfaces::msg::Time;
use r2r::geometry_msgs::msg::PoseStamped;
use r2r::nav_msgs::msg::Path;
use r2r::std_msgs::msg::Header;
use r2r::RosParams;
use tf_r2r::{TfError, TfListener};
use thiserror::Error;
use tokio::sync::Mutex;

#[derive(RosParams, Default, Debug)]
pub struct MapParams {
    // The frame in which the path planning is done
    pub frame: String,
    // The diameter of the ball
    pub ball_diameter: f32,
    // How do we want to inflate object sizes
    pub inflation: InflationParams,
}

#[derive(RosParams, Default, Debug)]
pub struct InflationParams {
    // Radius of a circle on the ground that represents the space occupied by our robot. Instead of planning with both a robot polygon/circle and an obstacle polygon, we just inflate the obstacles and assume the robot is a point. This is faster and simpler
    pub robot_radius: f64,
    // Distance we want to keep to obstacles when planning a path around them. No immediate action is required if the robot is closer than this distance to an obstacle, but we don't consider paths this close during the visibility graph generation.
    pub obstacle_margin: f64,
}

pub struct Planner {
    tf: Arc<Mutex<TfListener>>,
    goal: Option<PoseStamped>,
    ball_obstacle_active: bool,
    params: Arc<StdMutex<Params>>,
}

#[derive(Error, Debug)]
pub enum PlannerStepError {
    #[error("Could not determine the robot location")]
    LocalizationError(#[from] TfError),
    #[error("No goal was given when step was called")]
    NoGoalError(),
}

impl Planner {
    pub fn new(tf: Arc<Mutex<TfListener>>, params: Arc<StdMutex<Params>>) -> Self {
        Self {
            tf,
            goal: None,
            ball_obstacle_active: false,
            params,
        }
    }

    // Determine if we have an active goal
    pub fn active(&self) -> bool {
        self.goal.is_some()
    }

    // Removes the current goal
    pub fn cancel_goal(&mut self) {
        self.goal = None;
    }

    // Updates the goal pose
    pub fn set_goal(&mut self, goal: PoseStamped) {
        let mut goal = goal.clone();
        goal.header.stamp = Time::default(); // Null the time so we always use the latest
        self.goal = Some(goal);
    }

    // Activates or deactivates the ball obstacle
    pub fn avoid_ball(&mut self, state: bool) {
        self.ball_obstacle_active = state;
    }

    // Compute the next path to the goal
    pub async fn step(&self) -> Result<Path, PlannerStepError> {
        // Check if we have a goal
        let goal = self.goal.as_ref().ok_or(PlannerStepError::NoGoalError())?;

        // Get our current position
        let my_position = self
            .tf
            .lock()
            .await
            .lookup_transform(
                &self.params.lock().unwrap().map.frame,
                &self.params.lock().unwrap().base_footprint_frame,
                Time::default(),
            )
            .map_err(|e| PlannerStepError::LocalizationError(e))?
            .transform
            .translation;

        let map_config = ObstacleMapConfig::new(
            self.params.lock().unwrap().map.inflation.robot_radius,
            self.params.lock().unwrap().map.inflation.obstacle_margin,
            12,
        );

        let obstacles = vec![];
        // TODO add robot to map

        // TODO add ball to map

        let path = ObstacleMap::new(map_config, obstacles)
            .shortest_path(
                (my_position.x, my_position.y),
                (goal.pose.position.x, goal.pose.position.x),
            )
            .iter()
            .map(|position| {
                let mut pose = PoseStamped::default();
                pose.pose.position.x = position.0;
                pose.pose.position.y = position.1;
                pose
            })
            .chain([goal.clone()])
            .collect();

        return Ok(Path {
            header: Header {
                stamp: Time::default(),
                frame_id: self.params.lock().unwrap().map.frame.clone(),
            },
            poses: path,
        });
    }
}
