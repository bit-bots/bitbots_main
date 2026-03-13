use r2r;
use std::collections::HashMap;
use std::env;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::time::interval;
use tokio_stream::StreamExt;

pub mod droidgrpc {
    tonic::include_proto!("droidgrpc");
}

use droidgrpc::Empty;
// Note: Ensure these matches your generated code exactly
use droidgrpc::arm_service_client::ArmServiceClient;
use droidgrpc::leg_service_client::LegServiceClient;

// This struct holds the latest known positions for all joints
#[derive(Default)]
struct RobotState {
    // Key: ROS joint name, Value: position
    joint_data: HashMap<String, f64>,
}
#[derive(Default)]
struct GoalState {
    // Key: ROS joint name, Value: position target
    joint_targets: HashMap<String, f32>,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    let ip = args.get(1).map(|s| s.as_str()).unwrap_or("127.0.0.1");

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "x02_joint_control", "")?;
    let state_pub = node.create_publisher::<r2r::sensor_msgs::msg::JointState>(
        "/joint_states",
        r2r::QosProfile::default(),
    )?;
    let mut cmd_sub = node.subscribe::<r2r::bitbots_msgs::msg::JointCommand>(
        "/x02_joint_commands",
        r2r::QosProfile::default()
    )?;
    let state_pub = Arc::new(Mutex::new(state_pub));
    let spin_node = Arc::new(Mutex::new(node));

    // --- 1. Mapping & Virtual Joints ---
    let mut name_map = HashMap::new();
    let mappings = [
        ("muT", "LHipYaw"),
        ("msL", "LHipRoll"),
        ("mhL", "LHipPitch"),
        ("mkL", "LKnee"),
        ("maL", "LAnklePitch"),
        ("mbT", "RHipYaw"),
        ("msR", "RHipRoll"),
        ("mhR", "RHipPitch"),
        ("mkR", "RKnee"),
        ("maR", "RAnklePitch"),
        ("UL1", "LShoulderPitch"),
        ("UL2", "LShoulderRoll"),
        ("UL3", "LShoulderYaw"),
        ("UL4", "LElbow"),
        ("UR1", "RShoulderPitch"),
        ("UR2", "RShoulderRoll"),
        ("UR3", "RShoulderYaw"),
        ("UR4", "RElbow"),
    ];
    for (hw, ros) in mappings {
        name_map.insert(hw.to_string(), ros.to_string());
    }

    let virtual_joints = ["torso"];

    // --- 2. Shared State Initialization ---
    let mut initial_state = RobotState::default();
    mappings
        .iter()
        .map(|(_, ros)| ros)
        .chain(virtual_joints.iter())
        .for_each(|&name| {
            initial_state.joint_data.insert(name.to_string(), 0.0);
        });
    let shared_state = Arc::new(Mutex::new(initial_state));

    // --- 3. gRPC Setup & Config ---
    let mut leg_client = LegServiceClient::connect(format!("http://{}:50051", ip)).await?;
    let mut arm_client = ArmServiceClient::connect(format!("http://{}:50052", ip)).await?;

    let leg_joint_names = leg_client
        .get_leg_config(Empty {})
        .await?
        .into_inner()
        .joint_name;
    let arm_joint_names = arm_client
        .get_arm_config(Empty {})
        .await?
        .into_inner()
        .joint_name;

    let mut initial_goals = GoalState::default();

    // Seed Leg goals
    if let Ok(resp) = leg_client.get_leg_state(Empty {}).await {
        let current_pos = resp.into_inner().position;
        for (i, hw_id) in leg_joint_names.iter().enumerate() {
            if let Some(ros_name) = name_map.get(hw_id) {
                initial_goals.joint_targets.insert(ros_name.clone(), current_pos.get(i).cloned().unwrap_or(0.0));
            }
        }
    }

    // Seed Arm goals
    if let Ok(resp) = arm_client.get_arm_state(Empty {}).await {
        let current_pos = resp.into_inner().position;
        for (i, hw_id) in arm_joint_names.iter().enumerate() {
            if let Some(ros_name) = name_map.get(hw_id) {
                initial_goals.joint_targets.insert(ros_name.clone(), current_pos.get(i).cloned().unwrap_or(0.0));
            }
        }
    }

    let shared_goals = Arc::new(Mutex::new(initial_goals));

    // --- GPRC Tasks: Update shared state when data arrives, send out commands ---

    // Leg Poller
    let mut l_client = leg_client.clone();
    let l_state = shared_state.clone();
    let l_names = leg_joint_names.clone();
    let l_map = name_map.clone();
    let l_goals = shared_goals.clone();
    tokio::spawn(async move {
        let mut ticker = interval(Duration::from_millis(20)); // Poll at 50Hz
        loop {
            ticker.tick().await;
            if let Ok(resp) = l_client.get_leg_state(Empty {}).await {
                let positions = resp.into_inner().position;
                let mut state = l_state.lock().unwrap();
                for (i, hw_id) in l_names.iter().enumerate() {
                    if let Some(ros_name) = l_map.get(hw_id) {
                        state.joint_data.insert(
                            ros_name.clone(),
                            positions.get(i).cloned().unwrap_or(0.0) as f64,
                        );
                    }
                }
            }
            let cmd_request = {
                let goals = l_goals.lock().unwrap();
                let positions: Vec<f32> = l_names.iter()
                    .map(|hw_id| {
                        let ros_name = l_map.get(hw_id).expect("Map error");
                        *goals.joint_targets.get(ros_name).expect("Leg Joint has no value assigned")
                    })
                    .collect();

                droidgrpc::DroidCommandRequest {
                    position: positions,
                    ..Default::default()
                }
            };
            if let Err(e) = l_client.set_leg_command(cmd_request).await{
                eprintln!("gRPC Error: Failed to set Arm command: {}", e);
            }
        }
    });

    // Arm Poller
    let mut a_client = arm_client.clone();
    let a_state = shared_state.clone();
    let a_names = arm_joint_names.clone();
    let a_map = name_map.clone();
    let a_goals = shared_goals.clone();
    tokio::spawn(async move {
        let mut ticker = interval(Duration::from_millis(20));
        loop {
            let cmd_request = {
                let goals = a_goals.lock().unwrap();
                let positions: Vec<f32> = a_names.iter()
                    .map(|hw_id| {
                        let ros_name = a_map.get(hw_id).expect("Arm mapping error");
                        *goals.joint_targets.get(ros_name).expect("Arm Joint has no value assigned")
                    })
                    .collect();

                droidgrpc::DroidCommandRequest {
                    position: positions,
                    ..Default::default()
                }
            };

            if let Err(e) = a_client.set_arm_command(cmd_request).await {
                eprintln!("gRPC Error: Failed to set Arm command: {}", e);
            }
            ticker.tick().await;
            if let Ok(resp) = a_client.get_arm_state(Empty {}).await {
                let positions = resp.into_inner().position;
                let mut state = a_state.lock().unwrap();
                for (i, hw_id) in a_names.iter().enumerate() {
                    if let Some(ros_name) = a_map.get(hw_id) {
                        state.joint_data.insert(
                            ros_name.clone(),
                            positions.get(i).cloned().unwrap_or(0.0) as f64,
                        );
                    }
                }
            }
        }
    });

    // --- Reader Task: Publish at Fixed ROS Rate ---
    let read_node = spin_node.clone();
    let pub_state = shared_state.clone();
    tokio::spawn(async move {
        let mut ticker = interval(Duration::from_millis(10)); // Publish at 100Hz
        loop {
            ticker.tick().await;
            let res: Result<(), Box<dyn std::error::Error + Send + Sync>> = (|| async {
                let mut ros_msg = r2r::sensor_msgs::msg::JointState::default();
                // Get the duration from the clock
                let now_duration = {
                    let n_lock = read_node.lock().expect("node lock poisoned");
                    let clock_res = n_lock.get_ros_clock();
                    let mut c_lock = clock_res.lock().expect("clock lock poisoned");
                    c_lock.get_now().expect("Clock failed")
                };

                // Convert Duration to ROS Time message
                ros_msg.header.stamp = r2r::builtin_interfaces::msg::Time {
                    sec: now_duration.as_secs() as i32,
                    nanosec: now_duration.subsec_nanos(),
                };

                // Lock briefly to copy data
                {
                    let state = pub_state.lock().unwrap();
                    for (name, pos) in &state.joint_data {
                        ros_msg.name.push(name.clone());
                        ros_msg.position.push(*pos);
                    }
                }

                // Always add virtual joints (ensures consistency even before 1st gRPC response)
                for v_joint in virtual_joints {
                    if !ros_msg.name.contains(&v_joint.to_string()) {
                        ros_msg.name.push(v_joint.to_string());
                        ros_msg.position.push(0.0);
                    }
                }

                if !ros_msg.name.is_empty() {
                    let _ = state_pub
                        .lock()
                        .expect("Publisher Poisoned")
                        .publish(&ros_msg);
                }
                Ok(())
            })()
            .await;

            if let Err(e) = res {
                eprintln!("Publisher loop error: {}", e);
            }
        }
    });
    // ---ROS receive goals---
    let sub_goals = shared_goals.clone();

    tokio::spawn(async move {
        while let Some(msg) = cmd_sub.next().await {
            if msg.joint_names.len() != msg.positions.len() { continue; }

            let mut goals = sub_goals.lock().unwrap();
            for (i, ros_name) in msg.joint_names.iter().enumerate() {
                // Only update if it's a joint we actually track
                if goals.joint_targets.contains_key(ros_name) {
                    goals.joint_targets.insert(ros_name.clone(), msg.positions[i] as f32);
                }
            }
        }
    });

    // ---ROS Spin ---
    loop {
        spin_node
            .lock()
            .expect("Node died")
            .spin_once(std::time::Duration::from_millis(100));
    }
}
