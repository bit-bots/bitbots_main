use std::sync::{Arc, Mutex as StdMutex};
use tokio::sync::Mutex;

use crate::controller::{Controller, ControllerParams};
use crate::planner::{MapParams, Planner};
use futures::StreamExt;
use r2r::geometry_msgs::msg::{PointStamped, PoseStamped, PoseWithCovarianceStamped, Twist};
use r2r::nav_msgs::msg::Path;
use r2r::soccer_vision_3d_msgs::msg::RobotArray;
use r2r::std_msgs::msg::{Bool, Empty};
use r2r::visualization_msgs::msg::MarkerArray;
use r2r::RosParams;

mod controller;
mod planner;

#[derive(RosParams, Default, Debug)]
pub struct Params {
    // The rate at which the path planning is executed
    pub rate: f32,
    // The frame of the robot base
    pub base_footprint_frame: String,
    // Map related parameters
    pub map: MapParams,
    // Controller related parameters
    pub controller: ControllerParams,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;

    // Create a node
    let mut node = r2r::Node::create(ctx, "bitbots_path_planning", "")?;

    // Get a logger name for easier logging
    let nl = node.logger().to_string();

    // Parameter handling
    let params = Arc::new(StdMutex::new(Params::default()));
    let (paramater_handler, _) = node.make_derived_parameter_handler(params.clone())?;
    tokio::task::spawn(paramater_handler);

    // Create subscribers, publishers, services, timers, etc.
    let mut ball_sub = node
        .subscribe::<PoseWithCovarianceStamped>(
            "ball_position_relative_filtered",
            r2r::QosProfile::default(),
        )?
        .fuse();
    let mut robot_sub = node
        .subscribe::<RobotArray>("robots_relative_filtered", r2r::QosProfile::default())?
        .fuse();
    let mut goal_sub = node
        .subscribe::<PoseStamped>("goal_pose", r2r::QosProfile::default())?
        .fuse();
    let mut cancel_sub = node
        .subscribe::<Empty>("pathfinding/cancel", r2r::QosProfile::default())?
        .fuse();
    let mut avoid_ball_sub = node
        .subscribe::<Bool>("ball_obstacle_active", r2r::QosProfile::default())?
        .fuse();

    let cmd_vel_pub = node.create_publisher::<Twist>("cmd_vel", r2r::QosProfile::default())?;
    let path_pub = node.create_publisher::<Path>("path", r2r::QosProfile::default())?;
    let carrot_pub = node.create_publisher::<PointStamped>("carrot", r2r::QosProfile::default())?;
    let graph_pub =
        node.create_publisher::<MarkerArray>("visibility_graph", r2r::QosProfile::default())?;

    let mut timer = node.create_timer(std::time::Duration::from_secs_f32(
        1. / params.lock().unwrap().rate,
    ))?;

    let tf_listener = Arc::new(Mutex::new(tf_r2r::TfListener::new(&mut node)));

    // Share the node between tasks / threads
    let node = Arc::new(Mutex::new(node));
    let spin_node = node.clone();

    tokio::task::spawn(async move {
        // Define a thread-local state
        let mut planner = Planner::new(tf_listener.clone(), params.clone());
        let controller = Controller::new(tf_listener.clone(), params.clone());

        // Handle events
        loop {
            tokio::select! {
                _ = timer.tick() => {
                    if planner.active() {
                        // Calculate the path to the goal pose considering the current map
                        match planner.step().await {
                            Err(e) => {
                                r2r::log_warn!(&nl, "Planner step failed: {:}", e);
                            },
                            Ok(path) => {
                                // Publish the path for visualization
                                path_pub.publish(&path).expect("Publishing the path failed");
                                // Calculate the command velocity to follow the given path
                                match controller.step(&path).await {
                                    Err(e) => {
                                        r2r::log_warn!(&nl, "Controller step failed: {:}", e);
                                    },
                                    Ok((cmd_vel, carrot_point)) => {
                                        // Publish the walk command to control the robot
                                        cmd_vel_pub.publish(&cmd_vel).expect("Publishing command velocity failed");
                                        // Publish the carrot point for visualization
                                        carrot_pub.publish(&carrot_point).expect("Publishing carrot failed");
                                    }
                                }
                            }
                        }
                    }
                },
                Some(msg) = goal_sub.next() => {
                    planner.set_goal(msg);
                },
                _ = cancel_sub.next() => {
                    planner.cancel_goal();
                }
                Some(msg) = avoid_ball_sub.next() => {
                    planner.avoid_ball(msg.data);
                }
            }
        }
    });

    // Spin the underlying rcl node object
    loop {
        spin_node
            .lock()
            .await
            .spin_once(std::time::Duration::from_millis(100));
    }
}
