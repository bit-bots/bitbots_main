use std::env;
use droidgrpc::Empty;
use droidgrpc::arm_service_client::ArmServiceClient;
use droidgrpc::leg_service_client::LegServiceClient;

pub mod droidgrpc {
    tonic::include_proto!("droidgrpc");
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Get IP from CLI argument or default to localhost
    let args: Vec<String> = env::args().collect();
    let ip_str = args.get(1).map(|s| s.as_str()).unwrap_or("127.0.0.1");

    // Parse the string into an IpAddr to ensure it's a valid IPv4/IPv6
    let leg_uri = format!("http://{}:50051", ip_str);
    let arm_uri = format!("http://{}:50052", ip_str);
    let mut leg_client = LegServiceClient::connect(leg_uri).await?;
    let mut arm_client = ArmServiceClient::connect(arm_uri).await?;

    let request = tonic::Request::new(Empty {});

    let leg_config = leg_client.get_leg_config(request).await?;
    let request = tonic::Request::new(Empty {});
    let arm_config = arm_client.get_arm_config(request).await?;

    println!("LEG_CONFIG={leg_config:?}");
    println!("ARM_CONFIG={arm_config:?}");

    Ok(())
}