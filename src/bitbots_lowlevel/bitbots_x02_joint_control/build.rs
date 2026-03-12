fn main() -> Result<(), Box<dyn std::error::Error>> {
    tonic_prost_build::configure()
        .compile_protos(
            &[
                "proto/droid_msg.proto",
                "proto/leg_service.proto",
                "proto/arm_service.proto",
            ],
            &["proto"],
        )
        .unwrap();
    Ok(())
}
