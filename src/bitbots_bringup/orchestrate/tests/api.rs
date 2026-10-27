//! Talks to a real server on a temporary socket through the client, using the shipped
//! teamplayer stack file. No component is started.

use std::sync::{Arc, Mutex};

use bitbots_orchestrate::{
    api::{GlobalValue, LogStream, SetGlobalParameters, SetParameters, Value},
    client::Client,
    logs::LogStore,
    server::{self, ServerState},
    supervisor::Supervisor,
};
use bitbots_stack::Stack;

/// Source directory of a workspace package, which has the layout of its share directory for
/// the configuration used here.
fn workspace_package(name: &str) -> Option<std::path::PathBuf> {
    fn walk(dir: &std::path::Path, name: &str) -> Option<std::path::PathBuf> {
        if std::fs::read_to_string(dir.join("package.xml"))
            .is_ok_and(|xml| xml.contains(&format!("<name>{name}</name>")))
        {
            return Some(dir.to_path_buf());
        }
        for entry in std::fs::read_dir(dir).ok()?.flatten() {
            let path = entry.path();
            if path.is_dir() && !entry.file_name().to_string_lossy().starts_with('.') {
                if let Some(found) = walk(&path, name) {
                    return Some(found);
                }
            }
        }
        None
    }
    walk(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."),
        name,
    )
}

fn temp_socket(test: &str) -> std::path::PathBuf {
    std::env::temp_dir()
        .join(format!("bitbots_orchestrate_test_{}", std::process::id()))
        .join(format!("{test}.sock"))
}

async fn serve(test: &str) -> (Client, Arc<LogStore>, server::SocketGuard) {
    let socket = temp_socket(test);
    let (listener, guard) = server::bind(&socket).unwrap();
    let logs = Arc::new(LogStore::new());
    let stack_file =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../config/teamplayer.yaml");
    let stack = Stack::load(&stack_file).unwrap();
    let share = Box::new(|package: &str| {
        workspace_package(package).ok_or_else(|| anyhow::anyhow!("{package} not in the workspace"))
    });
    let router = server::router(ServerState {
        supervisor: Arc::new(Mutex::new(
            Supervisor::new(
                stack,
                logs.clone(),
                share,
                socket.with_extension("global_parameters.yaml"),
            )
            .unwrap(),
        )),
        logs: logs.clone(),
    });
    tokio::spawn(async move { axum::serve(listener, router).await });
    (Client::new(socket), logs, guard)
}

#[tokio::test]
async fn parameters_profiles_and_selection_round_trip() {
    let (client, _logs, _guard) = serve("config").await;
    let find = |status: &bitbots_orchestrate::api::Status, key: &str| {
        status
            .components
            .iter()
            .find(|c| c.key == key)
            .unwrap()
            .clone()
    };

    let status = client.status().await.unwrap();
    assert!(!find(&status, "tts").enabled);
    assert!(status.problems.is_empty(), "{:?}", status.problems);

    let status = client.set_enabled("tts", true).await.unwrap();
    assert!(find(&status, "tts").active);

    let status = client
        .set_parameters(&SetParameters::from([(
            "dsd_file".to_string(),
            Value::String("other.dsd".into()),
        )]))
        .await
        .unwrap();
    let dsd = status
        .parameters
        .iter()
        .find(|p| p.name == "dsd_file")
        .unwrap();
    assert_eq!(dsd.value, Value::String("other.dsd".into()));

    let status = client.apply_profile("sim").await.unwrap();
    assert_eq!(status.profile.as_deref(), Some("sim"));
    assert!(find(&status, "simulator").active);
    assert!(!find(&status, "lowlevel").available);

    // The fallback takes over while the localization is off.
    assert!(!find(&status, "fake_localization").active);
    let status = client.set_enabled("localization", false).await.unwrap();
    assert!(find(&status, "fake_localization").active);
}

#[tokio::test]
async fn errors_carry_status_and_message() {
    let (client, _logs, _guard) = serve("errors").await;
    let err = |r: anyhow::Result<_>| format!("{:#}", r.expect_err("must fail"));

    let e = err(client.start("nope").await);
    assert!(
        e.contains("404") && e.contains("unknown component key"),
        "{e}"
    );

    let e = err(client
        .set_parameters(&SetParameters::from([(
            "sim".to_string(),
            Value::String("yes".into()),
        )]))
        .await);
    assert!(e.contains("400") && e.contains("sim"), "{e}");

    let e = err(client.apply_profile("nope").await);
    assert!(e.contains("400") && e.contains("unknown profile"), "{e}");

    let e = err(client.set_enabled("robot_description", false).await);
    assert!(e.contains("409") && e.contains("locked"), "{e}");
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

#[tokio::test]
async fn fields_are_offered_as_choices() {
    let (client, _logs, _guard) = serve("fields").await;
    let status = client.status().await.unwrap();
    let field = status
        .parameters
        .iter()
        .find(|p| p.name == "fieldname")
        .unwrap();
    let choices = field.choices.as_ref().expect("fields are discovered");
    assert!(choices.contains(&"labor".to_string()) && choices.contains(&"hsl_kid".to_string()));

    let set =
        |field: &str| SetParameters::from([("fieldname".to_string(), Value::String(field.into()))]);
    client.set_parameters(&set("hsl_kid")).await.unwrap();
    let e = format!(
        "{:#}",
        client.set_parameters(&set("nope")).await.unwrap_err()
    );
    assert!(e.contains("400") && e.contains("choose one of"), "{e}");
}

#[tokio::test]
async fn global_parameters_are_written_for_the_next_start() {
    let (client, _logs, _guard) = serve("globals").await;
    let file = client.socket().with_extension("global_parameters.yaml");
    assert!(client.status().await.unwrap().global_parameters.is_empty());
    let no_overrides = std::fs::read_to_string(&file).unwrap();
    assert!(
        no_overrides.contains("ros__parameters: {}"),
        "{no_overrides}"
    );

    let status = client
        .set_global_parameters(&SetGlobalParameters::from([(
            "bot_id".to_string(),
            Some(GlobalValue::Integer(4)),
        )]))
        .await
        .unwrap();
    assert_eq!(status.global_parameters["bot_id"], GlobalValue::Integer(4));
    let text = std::fs::read_to_string(&file).unwrap();
    assert!(
        text.contains("ros__parameters") && text.contains("bot_id: 4"),
        "{text}"
    );

    let e = format!(
        "{:#}",
        client
            .set_global_parameters(&SetGlobalParameters::from([(
                "not valid".to_string(),
                Some(GlobalValue::Integer(1)),
            )]))
            .await
            .unwrap_err()
    );
    assert!(
        e.contains("400") && e.contains("no valid ROS parameter name"),
        "{e}"
    );

    let status = client
        .set_global_parameters(&SetGlobalParameters::from([("bot_id".to_string(), None)]))
        .await
        .unwrap();
    assert!(status.global_parameters.is_empty());
    assert_eq!(std::fs::read_to_string(&file).unwrap(), no_overrides);
}
