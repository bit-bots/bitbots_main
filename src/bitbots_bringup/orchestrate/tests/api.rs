//! Talks to a real server on a temporary socket through the client.

use std::sync::{Arc, Mutex};

use bitbots_orchestrate::{
    api::{ConfigUpdate, LogStream},
    client::Client,
    logs::LogStore,
    server::{self, ServerState},
    supervisor::Supervisor,
};

fn temp_socket(test: &str) -> std::path::PathBuf {
    std::env::temp_dir()
        .join(format!("bitbots_orchestrate_test_{}", std::process::id()))
        .join(format!("{test}.sock"))
}

async fn serve(test: &str) -> (Client, Arc<LogStore>, server::SocketGuard) {
    let socket = temp_socket(test);
    let (listener, guard) = server::bind(&socket).unwrap();
    let logs = Arc::new(LogStore::new());
    let router = server::router(ServerState {
        supervisor: Arc::new(Mutex::new(Supervisor::new(logs.clone()))),
        logs: logs.clone(),
    });
    tokio::spawn(async move { axum::serve(listener, router).await });
    (Client::new(socket), logs, guard)
}

#[tokio::test]
async fn config_and_enabled_round_trip() {
    let (client, _logs, _guard) = serve("config").await;

    let status = client.status().await.unwrap();
    let tts = status.components.iter().find(|c| c.key == "tts").unwrap();
    assert!(!tts.enabled);

    let status = client.set_enabled("tts", true).await.unwrap();
    assert!(status
        .components
        .iter()
        .any(|c| c.key == "tts" && c.enabled));

    let status = client
        .update_config(&ConfigUpdate {
            dsd_file: Some("other.dsd".into()),
            ..Default::default()
        })
        .await
        .unwrap();
    assert_eq!(status.config.dsd_file, "other.dsd");
}

#[tokio::test]
async fn errors_carry_status_and_message() {
    let (client, _logs, _guard) = serve("errors").await;

    let err = client.start("nope").await.unwrap_err().to_string();
    assert!(
        err.contains("404") && err.contains("unknown component key"),
        "{err}"
    );

    let err = client
        .update_config(&ConfigUpdate {
            fieldname: Some(String::new()),
            ..Default::default()
        })
        .await
        .unwrap_err()
        .to_string();
    assert!(err.contains("400") && err.contains("fieldname"), "{err}");
}

#[tokio::test]
async fn logs_are_served_incrementally() {
    let (client, logs, _guard) = serve("logs").await;
    logs.push("vision", "Vision", LogStream::Stdout, "first");
    logs.push("ipm", "IPM", LogStream::Stderr, "second");

    let all = client.logs(0, None).await.unwrap();
    assert_eq!(all.lines.len(), 2);
    let newer = client.logs(all.lines[0].seq, None).await.unwrap();
    assert_eq!(newer.lines.len(), 1);
    assert_eq!(newer.lines[0].text, "second");

    let vision = client.logs(0, Some("vision")).await.unwrap();
    assert_eq!(vision.lines.len(), 1);
    assert!(client.logs(0, Some("nope")).await.is_err());
}

#[tokio::test]
async fn second_instance_on_the_same_socket_is_refused() {
    let (client, _logs, _guard) = serve("exclusive").await;
    let err = match server::bind(client.socket()) {
        Ok(_) => panic!("a second bind of the same socket must fail"),
        Err(e) => e.to_string(),
    };
    assert!(err.contains("already serves"), "{err}");
    // The running instance keeps working.
    client.status().await.unwrap();
}

#[tokio::test]
async fn missing_server_is_reported() {
    let client = Client::new(temp_socket("missing"));
    let err = format!("{:#}", client.status().await.unwrap_err());
    assert!(err.contains("cannot connect"), "{err}");
}
