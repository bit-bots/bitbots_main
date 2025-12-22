use std::sync::{Arc, Mutex as StdMutex};
use tokio::sync::Mutex;

use futures::StreamExt;
use r2r::bitbots_msgs::msg::Workload;
use r2r::bitbots_msgs::srv::Leds;
use r2r::std_msgs::msg::Empty;
use r2r::RosParams;

#[derive(RosParams, Default, Debug)]
struct Params {
    /// Parameter description (Yes this comment will appear in the parameter description)
    par1: f64,
    /// Dummy parameter [m/s]
    par2: i32,
    nested: NestedParams,
}

#[derive(RosParams, Default, Debug)]
struct NestedParams {
    par3: String,
    par4: u16,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;

    // Create a node
    let mut node = r2r::Node::create(ctx, "test_node", "")?;

    // Get a logger name for easier logging
    let nl = node.logger().to_string();

    // Create subscribers, publishers, services, timers, etc.
    let mut sub = node
        .subscribe::<Empty>("/hello_world", r2r::QosProfile::default())?
        .fuse();
    let mut timer = node.create_timer(std::time::Duration::from_secs_f32(0.05))?;
    let mut timer1 = node.create_timer(std::time::Duration::from_secs_f32(0.1))?;
    let publ = node.create_publisher::<Empty>("/out", r2r::QosProfile::default())?;
    let mut srv = node
        .create_service::<Leds::Service>("/led", r2r::QosProfile::default())?
        .fuse();

    // Parameter handling
    let params = Arc::new(StdMutex::new(Params::default()));
    let (paramater_handler, _) =
        node.make_derived_parameter_handler(params.clone())?;
    tokio::task::spawn(paramater_handler);

    println!("Sim time is set to: {}", node.get_parameter::<bool>("use_sim_time").unwrap());

    // Share the node between tasks / threads
    let node = Arc::new(Mutex::new(node));
    let spin_node = node.clone();

    tokio::task::spawn(async move {
        // Define a thread-local state
        let mut state = Workload::default();

        // Handle events
        loop {
            tokio::select! {
                req = srv.next() => {
                    let req = req.unwrap();
                    r2r::log_info!(&nl, "leds: {:?}", req.message);
                    // Flush the stdout buffer to ensure the print appears in the logs
                    let resp = Leds::Response::default();
                    req.respond(resp).unwrap();
                },
                msg = sub.next() => {
                    r2r::log_info!(&nl, "message: {:?}", msg);
                    // Flush the stdout buffer to ensure the print appears in the logs
                    state.memory_used += 1;
                },
                _ = timer.tick() => {
                    publ.publish(&Empty {}).inspect_err(|e| {
                        r2r::log_error!(&nl, "Error publishing message: {}", e);
                    }).ok();
                    r2r::log_info!(&nl, "timer tick {}", state.memory_used);
                },
                _ = timer1.tick() => {
                    let my_param = node.lock().await.get_parameter::<f64>("par1").unwrap();
                    r2r::log_info!(&nl, "Parameter par1: {}", my_param);
                    r2r::log_info!(&nl, "timer1 tick {}", state.memory_used);
                },
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
