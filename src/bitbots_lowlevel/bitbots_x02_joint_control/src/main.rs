use r2r;
use std::collections::HashMap;
use std::env;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::time::interval;

pub mod droidgrpc {
    tonic::include_proto!("droidgrpc");
}

use droidgrpc::{Empty};
// Note: Ensure these matches your generated code exactly
use droidgrpc::arm_service_client::ArmServiceClient;
use droidgrpc::leg_service_client::LegServiceClient;

// This struct holds the latest known positions for all joints
#[derive(Default)]
struct RobotState {
    // Key: ROS joint name, Value: position
    joint_data: HashMap<String, f64>,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    let ip = args.get(1).map(|s| s.as_str()).unwrap_or("127.0.0.1");

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "x02_joint_control", "")?;
    let state_pub = node.create_publisher::<r2r::sensor_msgs::msg::JointState>("/joint_states", r2r::QosProfile::default())?;
    let state_pub = Arc::new(Mutex::new(state_pub));
    let node = Arc::new(Mutex::new(node));
    let spin_node = node.clone();

    // --- 1. Mapping & Virtual Joints ---
    let mut name_map = HashMap::new();
    let mappings = [
        ("muT", "LHipYaw"), ("msL", "LHipRoll"), ("mhL", "LHipPitch"), ("mkL", "LKnee"), ("maL", "LAnklePitch"),
        ("mbT", "RHipYaw"), ("msR", "RHipRoll"), ("mhR", "RHipPitch"), ("mkR", "RKnee"), ("maR", "RAnklePitch"),
        ("UL1", "LShoulderPitch"), ("UL2", "LShoulderRoll"), ("UL3", "LShoulderYaw"), ("UL4", "LElbow"),
        ("UR1", "RShoulderPitch"), ("UR2", "RShoulderRoll"), ("UR3", "RShoulderYaw"), ("UR4", "RElbow"),
    ];
    for (hw, ros) in mappings { name_map.insert(hw.to_string(), ros.to_string()); }

    let virtual_joints = ["torso"];

    // --- 2. Shared State Initialization ---
    let mut initial_state = RobotState::default();
    mappings.iter().map(|(_, ros)| ros).chain(virtual_joints.iter())
        .for_each(|&name| { initial_state.joint_data.insert(name.to_string(), 0.0); });
    let shared_state = Arc::new(Mutex::new(initial_state));

    // --- 3. gRPC Setup & Config ---
    let mut leg_client = LegServiceClient::connect(format!("http://{}:50051", ip)).await?;
    let mut arm_client = ArmServiceClient::connect(format!("http://{}:50052", ip)).await?;

    let leg_joint_names = leg_client.get_leg_config(Empty {}).await?.into_inner().joint_name;
    let arm_joint_names = arm_client.get_arm_config(Empty {}).await?.into_inner().joint_name;

    // --- 4. Writer Tasks: Update shared state when data arrives ---

    // Leg Poller
    let mut l_client = leg_client.clone();
    let l_state = shared_state.clone();
    let l_names = leg_joint_names.clone();
    let l_map = name_map.clone();
    tokio::spawn(async move {
        let mut ticker = interval(Duration::from_millis(20)); // Poll at 50Hz
        loop {
            ticker.tick().await;
            if let Ok(resp) = l_client.get_leg_state(Empty {}).await {
                let positions = resp.into_inner().position;
                let mut state = l_state.lock().unwrap();
                for (i, hw_id) in l_names.iter().enumerate() {
                    if let Some(ros_name) = l_map.get(hw_id) {
                        state.joint_data.insert(ros_name.clone(), positions.get(i).cloned().unwrap_or(0.0) as f64);
                    }
                }
            }
        }
    });

    // Arm Poller
    let mut a_client = arm_client.clone();
    let a_state = shared_state.clone();
    let a_names = arm_joint_names.clone();
    let a_map = name_map.clone();
    tokio::spawn(async move {
        let mut ticker = interval(Duration::from_millis(20));
        loop {
            ticker.tick().await;
            if let Ok(resp) = a_client.get_arm_state(Empty {}).await {
                let positions = resp.into_inner().position;
                let mut state = a_state.lock().unwrap();
                for (i, hw_id) in a_names.iter().enumerate() {
                    if let Some(ros_name) = a_map.get(hw_id) {
                        state.joint_data.insert(ros_name.clone(), positions.get(i).cloned().unwrap_or(0.0) as f64);
                    }
                }
            }
        }
    });

    // --- 5. Reader Task: Publish at Fixed ROS Rate ---
    let pub_state = shared_state.clone();
    tokio::spawn(async move {
        let mut ticker = interval(Duration::from_millis(10)); // Publish at 100Hz
        loop {
            ticker.tick().await;
            let res: Result<(), Box<dyn std::error::Error + Send + Sync>> = (|| async {
                let mut ros_msg = r2r::sensor_msgs::msg::JointState::default();
                // Get the duration from the clock
                let now_duration = {
                    let n_lock = node.lock().expect("node lock poisoned");
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
                    let _ = state_pub.lock().expect("Publisher Poisoned").publish(&ros_msg);
                }
                Ok(())
            })().await;

            if let Err(e) = res {
                eprintln!("Publisher loop error: {}", e);
            }
        }
    });


    // --- 6. ROS Spin ---
    loop {
        spin_node.lock().expect("Node died").spin_once(std::time::Duration::from_millis(100));
    }
}