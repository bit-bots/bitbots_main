//! Talks to a real server on a temporary socket through the client, using the shipped
//! teamplayer stack file. No component is started.

use std::sync::{Arc, Mutex};

use bitbots_orchestrate::{
    api::{GlobalSource, GlobalValue, LogStream, SetOverrides, SetParameters, Value},
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
            Supervisor::new(stack, logs.clone(), share).unwrap(),
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
async fn blackboard_is_assembled_from_the_field_and_settings() {
    let (client, _logs, _guard) = serve("blackboard").await;
    let status = client.status().await.unwrap();
    let field = status
        .parameters
        .iter()
        .find(|p| p.name == "fieldname")
        .unwrap();
    let choices = field.choices.as_ref().expect("fields are discovered");
    assert!(choices.contains(&"labor".to_string()) && choices.contains(&"hsl_kid".to_string()));

    let bb = client.blackboard().await.unwrap();
    assert_eq!(bb.summary.error, None);
    assert_eq!(bb.summary.node, "parameter_blackboard");
    let get = |bb: &bitbots_orchestrate::api::BlackboardInfo, name: &str| {
        bb.parameters
            .iter()
            .find(|p| p.name == name)
            .unwrap()
            .clone()
    };
    assert_eq!(
        get(&bb, "field.name").value,
        GlobalValue::String("labor".into())
    );
    assert!(matches!(
        get(&bb, "field.size.x").source,
        GlobalSource::File(_)
    ));
    assert!(matches!(get(&bb, "bot_id").value, GlobalValue::Integer(_)));
    assert!(bb.maps.contains(&"field/map/lines".to_string()));

    // Changing the field reloads its configuration.
    client
        .set_parameters(&SetParameters::from([(
            "fieldname".to_string(),
            Value::String("hsl_kid".into()),
        )]))
        .await
        .unwrap();
    let bb = client.blackboard().await.unwrap();
    assert_eq!(
        get(&bb, "field.name").value,
        GlobalValue::String("hsl_kid".into())
    );

    // Overrides, e.g. for one of several simulated robots, and their removal.
    let bb = client
        .set_overrides(&SetOverrides::from([(
            "bot_id".to_string(),
            Some(GlobalValue::Integer(4)),
        )]))
        .await
        .unwrap();
    assert_eq!(get(&bb, "bot_id").value, GlobalValue::Integer(4));
    assert_eq!(get(&bb, "bot_id").source, GlobalSource::Override);
    let bb = client
        .set_overrides(&SetOverrides::from([("bot_id".to_string(), None)]))
        .await
        .unwrap();
    assert!(matches!(get(&bb, "bot_id").source, GlobalSource::File(_)));
}

#[tokio::test]
async fn invalid_fields_and_overrides_are_refused() {
    let (client, _logs, _guard) = serve("blackboard_errors").await;
    fn err<T: std::fmt::Debug>(r: anyhow::Result<T>) -> String {
        format!("{:#}", r.expect_err("must fail"))
    }

    let e = err(client
        .set_parameters(&SetParameters::from([(
            "fieldname".to_string(),
            Value::String("nope".into()),
        )]))
        .await);
    assert!(e.contains("400") && e.contains("choose one of"), "{e}");

    let e = err(client
        .set_overrides(&SetOverrides::from([(
            "bot_id".to_string(),
            Some(GlobalValue::String("four".into())),
        )]))
        .await);
    assert!(e.contains("400") && e.contains("bot_id"), "{e}");

    let e = err(client
        .set_overrides(&SetOverrides::from([(
            "no_such_parameter".to_string(),
            Some(GlobalValue::Integer(1)),
        )]))
        .await);
    assert!(
        e.contains("400") && e.contains("unknown global parameter"),
        "{e}"
    );
    // A refused override leaves no trace.
    assert!(client
        .blackboard()
        .await
        .unwrap()
        .summary
        .overrides
        .is_empty());
}
