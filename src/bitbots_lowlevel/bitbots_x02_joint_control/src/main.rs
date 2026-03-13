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
use droidgrpc::arm_service_client::ArmServiceClient;
use droidgrpc::leg_service_client::LegServiceClient;

#[derive(Default)]
struct RobotState {
    joint_data: HashMap<String, f64>,
}

#[derive(Default)]
struct GoalState {
    joint_targets: HashMap<String, f32>,
}

struct DroidBridge {
    name_map: HashMap<String, String>,
    virtual_joints: Vec<String>,
    shared_state: Arc<Mutex<RobotState>>,
    shared_goals: Arc<Mutex<GoalState>>,
    node: Arc<Mutex<r2r::Node>>,
}

impl DroidBridge {
    fn new(node: r2r::Node, mappings: [(&str, &str); 18], virtual_joints: Vec<String>) -> Self {
        let mut name_map = HashMap::new();
        for (hw, ros) in mappings {
            name_map.insert(hw.to_string(), ros.to_string());
        }

        let mut initial_state = RobotState::default();
        mappings.iter().for_each(|(_, ros)| {
            initial_state.joint_data.insert(ros.to_string(), 0.0);
        });
        virtual_joints.iter().for_each(|name| {
            initial_state.joint_data.insert(name.to_string(), 0.0);
        });

        Self {
            name_map,
            virtual_joints,
            shared_state: Arc::new(Mutex::new(initial_state)),
            shared_goals: Arc::new(Mutex::new(GoalState::default())),
            node: Arc::new(Mutex::new(node)),
        }
    }

    async fn seed_initial_goals(
        &self,
        leg_client: &mut LegServiceClient<tonic::transport::Channel>,
        arm_client: &mut ArmServiceClient<tonic::transport::Channel>,
        leg_names: &[String],
        arm_names: &[String],
    ) {
        let mut goals = self.shared_goals.lock().unwrap();

        if let Ok(resp) = leg_client.get_leg_state(Empty {}).await {
            let pos = resp.into_inner().position;
            for (i, hw_id) in leg_names.iter().enumerate() {
                if let Some(ros_name) = self.name_map.get(hw_id) {
                    goals.joint_targets.insert(ros_name.clone(), pos[i]);
                }
            }
        }

        if let Ok(resp) = arm_client.get_arm_state(Empty {}).await {
            let pos = resp.into_inner().position;
            for (i, hw_id) in arm_names.iter().enumerate() {
                if let Some(ros_name) = self.name_map.get(hw_id) {
                    goals.joint_targets.insert(ros_name.clone(), pos[i]);
                }
            }
        }
    }

    fn run_publisher_task(&self, state_pub: Arc<Mutex<r2r::Publisher<r2r::sensor_msgs::msg::JointState>>>) {
        let node = self.node.clone();
        let pub_state = self.shared_state.clone();
        let virtuals = self.virtual_joints.clone();

        tokio::spawn(async move {
            let mut ticker = interval(Duration::from_millis(10));
            loop {
                ticker.tick().await;
                let res: Result<(), Box<dyn std::error::Error + Send + Sync>> = (|| async {
                    let mut ros_msg = r2r::sensor_msgs::msg::JointState::default();
                    let now_duration = {
                        let n_lock = node.lock().expect("node lock poisoned");
                        let clock_res = n_lock.get_ros_clock();
                        clock_res.lock().expect("clock lock poisoned").get_now().expect("Clock failed")
                    };

                    ros_msg.header.stamp = r2r::builtin_interfaces::msg::Time {
                        sec: now_duration.as_secs() as i32,
                        nanosec: now_duration.subsec_nanos(),
                    };

                    {
                        let state = pub_state.lock().unwrap();
                        for (name, pos) in &state.joint_data {
                            ros_msg.name.push(name.clone());
                            ros_msg.position.push(*pos);
                        }
                    }

                    for v_joint in &virtuals {
                        if !ros_msg.name.contains(v_joint) {
                            ros_msg.name.push(v_joint.clone());
                            ros_msg.position.push(0.0);
                        }
                    }

                    state_pub.lock().expect("Publisher Poisoned").publish(&ros_msg)?;
                    Ok(())
                })().await;

                if let Err(e) = res { eprintln!("Publisher loop error: {}", e); }
            }
        });
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    let ip = args.get(1).map(|s| s.as_str()).unwrap_or("192.168.254.100");

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "x02_joint_control", "")?;
    let state_pub = Arc::new(Mutex::new(node.create_publisher::<r2r::sensor_msgs::msg::JointState>("/joint_states", r2r::QosProfile::default())?));
    let mut cmd_sub = node.subscribe::<r2r::bitbots_msgs::msg::JointCommand>("/x02_joint_commands", r2r::QosProfile::default())?;

    let mappings = [
        ("muT", "LHipYaw"), ("msL", "LHipRoll"), ("mhL", "LHipPitch"), ("mkL", "LKnee"), ("maL", "LAnklePitch"),
        ("mbT", "RHipYaw"), ("msR", "RHipRoll"), ("mhR", "RHipPitch"), ("mkR", "RKnee"), ("maR", "RAnklePitch"),
        ("UL1", "LShoulderPitch"), ("UL2", "LShoulderRoll"), ("UL3", "LShoulderYaw"), ("UL4", "LElbow"),
        ("UR1", "RShoulderPitch"), ("UR2", "RShoulderRoll"), ("UR3", "RShoulderYaw"), ("UR4", "RElbow"),
    ];

    let bridge = DroidBridge::new(node, mappings, vec!["torso".to_string()]);

    let mut leg_client = LegServiceClient::connect(format!("http://{}:50051", ip)).await?;
    let mut arm_client = ArmServiceClient::connect(format!("http://{}:50052", ip)).await?;

    let leg_joint_names = leg_client.get_leg_config(Empty {}).await?.into_inner().joint_name;
    let arm_joint_names = arm_client.get_arm_config(Empty {}).await?.into_inner().joint_name;

    bridge.seed_initial_goals(&mut leg_client, &mut arm_client, &leg_joint_names, &arm_joint_names).await;

    // Leg Task
    let l_client = leg_client.clone();
    let l_names = leg_joint_names.clone();
    let l_state = bridge.shared_state.clone();
    let l_goals = bridge.shared_goals.clone();
    let l_map = bridge.name_map.clone();
    tokio::spawn(async move {
        let mut client = l_client;
        let mut ticker = interval(Duration::from_millis(20));
        loop {
            ticker.tick().await;
            if let Ok(resp) = client.get_leg_state(Empty {}).await {
                let pos = resp.into_inner().position;
                let mut state = l_state.lock().unwrap();
                for (i, hw_id) in l_names.iter().enumerate() {
                    if let Some(ros_name) = l_map.get(hw_id) {
                        state.joint_data.insert(ros_name.clone(), pos[i] as f64);
                    }
                }
            }
            let req = droidgrpc::DroidCommandRequest {
                position: l_names.iter().map(|id| *l_goals.lock().unwrap().joint_targets.get(&l_map[id]).unwrap()).collect(),
                ..Default::default()
            };
            let _ = client.set_leg_command(req).await;
        }
    });

    // Arm Task
    let a_client = arm_client.clone();
    let a_names = arm_joint_names.clone();
    let a_state = bridge.shared_state.clone();
    let a_goals = bridge.shared_goals.clone();
    let a_map = bridge.name_map.clone();
    tokio::spawn(async move {
        let mut client = a_client;
        let mut ticker = interval(Duration::from_millis(20));
        loop {
            ticker.tick().await;
            let req = droidgrpc::DroidCommandRequest {
                position: a_names.iter().map(|id| *a_goals.lock().unwrap().joint_targets.get(&a_map[id]).unwrap()).collect(),
                ..Default::default()
            };
            let _ = client.set_arm_command(req).await;
            if let Ok(resp) = client.get_arm_state(Empty {}).await {
                let pos = resp.into_inner().position;
                let mut state = a_state.lock().unwrap();
                for (i, hw_id) in a_names.iter().enumerate() {
                    if let Some(ros_name) = a_map.get(hw_id) {
                        state.joint_data.insert(ros_name.clone(), pos[i] as f64);
                    }
                }
            }
        }
    });

    bridge.run_publisher_task(state_pub);

    let sub_goals = bridge.shared_goals.clone();
    tokio::spawn(async move {
        while let Some(msg) = cmd_sub.next().await {
            if msg.joint_names.len() == msg.positions.len() {
                let mut goals = sub_goals.lock().unwrap();
                for (i, name) in msg.joint_names.iter().enumerate() {
                    if goals.joint_targets.contains_key(name) {
                        goals.joint_targets.insert(name.clone(), msg.positions[i] as f32);
                    }
                }
            }
        }
    });

    let spin_node = bridge.node.clone();
    loop {
        spin_node.lock().unwrap().spin_once(Duration::from_millis(100));
    }
}