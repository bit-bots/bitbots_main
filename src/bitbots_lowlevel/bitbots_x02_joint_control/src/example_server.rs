use futures_util::StreamExt;
use tokio_stream::wrappers::ReceiverStream;
use tonic::transport::Server;
use tonic::{Request, Response, Status};

pub mod droidgrpc {
    tonic::include_proto!("droidgrpc");
}

use droidgrpc::arm_service_server::{ArmService, ArmServiceServer};
use droidgrpc::leg_service_server::{LegService, LegServiceServer};
use droidgrpc::{DroidArmResponse, DroidCommandRequest, DroidConfigs, DroidStateResponse, Empty};

#[derive(Debug, Default)]
pub struct DummyArmService {}

#[tonic::async_trait]
impl ArmService for DummyArmService {
    // --- 1. Define the missing associated types ---
    type GetArmStateStreamStream = ReceiverStream<Result<DroidArmResponse, Status>>;
    type ExchangeArmControlStreamStream = ReceiverStream<Result<DroidArmResponse, Status>>;

    // --- 2. Implement the methods ---

    async fn get_arm_config(&self, _: Request<Empty>) -> Result<Response<DroidConfigs>, Status> {
        Ok(Response::new(DroidConfigs::default()))
    }

    async fn get_arm_state(&self, _: Request<Empty>) -> Result<Response<DroidArmResponse>, Status> {
        Ok(Response::new(DroidArmResponse::default()))
    }

    async fn get_arm_state_stream(
        &self,
        _: Request<Empty>,
    ) -> Result<Response<Self::GetArmStateStreamStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(4);
        // Minimal result: just one empty response
        let _ = tx.try_send(Ok(DroidArmResponse::default()));
        Ok(Response::new(ReceiverStream::new(rx)))
    }

    async fn set_arm_command(
        &self,
        _: Request<DroidCommandRequest>,
    ) -> Result<Response<Empty>, Status> {
        Ok(Response::new(Empty::default()))
    }

    async fn set_arm_command_stream(
        &self,
        request: Request<tonic::Streaming<DroidCommandRequest>>,
    ) -> Result<Response<Empty>, Status> {
        let mut stream = request.into_inner();
        while let Some(_cmd) = stream.next().await {
            // Echo/Process logic here
        }
        Ok(Response::new(Empty::default()))
    }

    async fn exchange_arm_control_stream(
        &self,
        request: Request<tonic::Streaming<DroidCommandRequest>>,
    ) -> Result<Response<Self::ExchangeArmControlStreamStream>, Status> {
        let mut in_stream = request.into_inner();
        let (tx, rx) = tokio::sync::mpsc::channel(128);

        tokio::spawn(async move {
            while let Some(Ok(cmd)) = in_stream.next().await {
                // Echoing the command fields back into a response
                let echo = DroidArmResponse {
                    position: cmd.position,
                    velocity: cmd.velocity,
                    torque: cmd.torque,
                    ..Default::default()
                };
                if tx.send(Ok(echo)).await.is_err() {
                    break;
                }
            }
        });

        Ok(Response::new(ReceiverStream::new(rx)))
    }
}

#[derive(Debug, Default)]
pub struct DummyLegService {}

#[tonic::async_trait]
impl LegService for DummyLegService {
    // Associated types required by the generated trait for streaming RPCs
    type GetLegStateStreamStream = ReceiverStream<Result<DroidStateResponse, Status>>;
    type ExchangeLegControlStreamStream = ReceiverStream<Result<DroidStateResponse, Status>>;

    async fn get_leg_config(&self, _: Request<Empty>) -> Result<Response<DroidConfigs>, Status> {
        Ok(Response::new(DroidConfigs::default()))
    }

    async fn get_leg_state(
        &self,
        _: Request<Empty>,
    ) -> Result<Response<DroidStateResponse>, Status> {
        Ok(Response::new(DroidStateResponse::default()))
    }

    async fn get_leg_state_stream(
        &self,
        _: Request<Empty>,
    ) -> Result<Response<Self::GetLegStateStreamStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(4);
        // Minimal pulse: send one default state
        let _ = tx.try_send(Ok(DroidStateResponse::default()));
        Ok(Response::new(ReceiverStream::new(rx)))
    }

    async fn set_leg_command(
        &self,
        _: Request<DroidCommandRequest>,
    ) -> Result<Response<Empty>, Status> {
        Ok(Response::new(Empty::default()))
    }

    async fn set_leg_command_stream(
        &self,
        request: Request<tonic::Streaming<DroidCommandRequest>>,
    ) -> Result<Response<Empty>, Status> {
        let mut stream = request.into_inner();
        while let Some(Ok(_cmd)) = stream.next().await {
            // Echo logic could be added here for processing
        }
        Ok(Response::new(Empty::default()))
    }

    async fn exchange_leg_control_stream(
        &self,
        request: Request<tonic::Streaming<DroidCommandRequest>>,
    ) -> Result<Response<Self::ExchangeLegControlStreamStream>, Status> {
        let mut in_stream = request.into_inner();
        let (tx, rx) = tokio::sync::mpsc::channel(128);

        tokio::spawn(async move {
            while let Some(Ok(cmd)) = in_stream.next().await {
                // Echo common fields from DroidCommandRequest to DroidStateResponse
                let echo = DroidStateResponse {
                    position: cmd.position,
                    velocity: cmd.velocity,
                    torque: cmd.torque,
                    ..Default::default()
                };
                if tx.send(Ok(echo)).await.is_err() {
                    break;
                }
            }
        });

        Ok(Response::new(ReceiverStream::new(rx)))
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Define the addresses
    let leg_addr = "127.0.0.1:50051".parse()?;
    let arm_addr = "127.0.0.1:50052".parse()?;

    println!("LegService starting on {}", leg_addr);
    println!("ArmService starting on {}", arm_addr);

    // 2. Spawn the LegService on port 50051
    let leg_handle = tokio::spawn(async move {
        let service = DummyLegService::default();
        Server::builder()
            .add_service(LegServiceServer::new(service))
            .serve(leg_addr)
            .await
            .expect("LegService server failed");
    });

    // 3. Spawn the ArmService on port 50052
    let arm_handle = tokio::spawn(async move {
        let service = DummyArmService::default();
        Server::builder()
            .add_service(ArmServiceServer::new(service))
            .serve(arm_addr)
            .await
            .expect("ArmService server failed");
    });

    // 4. Wait for both servers (they will run indefinitely)
    let _ = tokio::try_join!(leg_handle, arm_handle)?;

    Ok(())
}
