use std::{
    env, fs,
    os::unix::fs::PermissionsExt,
    path::{Path, PathBuf},
};

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use tokio::{
    io::{AsyncBufReadExt, AsyncWriteExt, BufReader},
    net::{UnixListener, UnixStream},
    sync::{broadcast, mpsc, oneshot},
};

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "command", rename_all = "kebab-case")]
pub enum ControlRequest {
    Status,
    Watch,
    SetComponents {
        components: Vec<String>,
    },
    SetConfig {
        fieldname: String,
        game_settings: Option<String>,
    },
    Start {
        target: String,
    },
    Stop {
        target: String,
    },
    Restart {
        target: String,
    },
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ComponentSnapshot {
    pub key: String,
    pub name: String,
    pub enabled: bool,
    pub state: String,
    pub pids: Vec<u32>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct StatusSnapshot {
    pub fieldname: String,
    pub sim: bool,
    pub components: Vec<ComponentSnapshot>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ControlResponse {
    Status { status: StatusSnapshot },
    Ok { message: String },
    Error { message: String },
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ControlEvent {
    Component {
        key: String,
        state: String,
        code: Option<i32>,
    },
    Log {
        component: String,
        stream: String,
        line: String,
    },
}

pub struct ControlCommand {
    pub request: ControlRequest,
    pub reply: oneshot::Sender<ControlResponse>,
}

pub fn socket_path() -> PathBuf {
    if let Ok(path) = env::var("TEAMPLAYER_SOCKET") {
        return PathBuf::from(path);
    }
    let runtime = env::var("XDG_RUNTIME_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            PathBuf::from(format!("/tmp/teamplayer-{}", unsafe {
                nix::libc::geteuid()
            }))
        });
    runtime.join("teamplayer.sock")
}

pub async fn start_server(
    path: PathBuf,
    command_tx: mpsc::UnboundedSender<ControlCommand>,
    event_tx: broadcast::Sender<ControlEvent>,
) -> Result<tokio::task::JoinHandle<()>> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    if path.exists() {
        fs::remove_file(&path)?;
    }
    let listener = UnixListener::bind(&path)?;
    fs::set_permissions(&path, fs::Permissions::from_mode(0o600))?;
    Ok(tokio::spawn(async move {
        loop {
            let Ok((stream, _)) = listener.accept().await else {
                break;
            };
            let command_tx = command_tx.clone();
            let event_rx = event_tx.subscribe();
            tokio::spawn(async move {
                if let Err(error) = handle_connection(stream, command_tx, event_rx).await {
                    eprintln!("teamplayer control connection failed: {error:#}");
                }
            });
        }
    }))
}

async fn handle_connection(
    stream: UnixStream,
    command_tx: mpsc::UnboundedSender<ControlCommand>,
    mut event_rx: broadcast::Receiver<ControlEvent>,
) -> Result<()> {
    let (reader, mut writer) = stream.into_split();
    let mut lines = BufReader::new(reader).lines();
    let Some(line) = lines.next_line().await? else {
        return Ok(());
    };
    let request: ControlRequest = serde_json::from_str(&line)?;
    let watching = matches!(request, ControlRequest::Watch);
    let (reply_tx, reply_rx) = oneshot::channel();
    command_tx
        .send(ControlCommand {
            request,
            reply: reply_tx,
        })
        .map_err(|_| anyhow::anyhow!("teamplayer is shutting down"))?;
    let response = reply_rx.await?;
    write_json_line(&mut writer, &response).await?;
    if watching {
        loop {
            match event_rx.recv().await {
                Ok(event) => write_json_line(&mut writer, &event).await?,
                Err(broadcast::error::RecvError::Lagged(_)) => continue,
                Err(broadcast::error::RecvError::Closed) => return Ok(()),
            }
        }
    }
    Ok(())
}

pub async fn request(path: &Path, request: &ControlRequest, watch: bool) -> Result<()> {
    let stream = UnixStream::connect(path)
        .await
        .with_context(|| format!("could not connect to {}", path.display()))?;
    let (reader, mut writer) = stream.into_split();
    write_json_line(&mut writer, request).await?;
    let mut lines = BufReader::new(reader).lines();
    let mut first = true;
    while let Some(line) = lines.next_line().await? {
        println!("{line}");
        if first {
            first = false;
            if let ControlResponse::Error { message } = serde_json::from_str(&line)? {
                return Err(anyhow::anyhow!(message));
            }
        }
        if !watch {
            break;
        }
    }
    Ok(())
}

async fn write_json_line<T: Serialize>(
    writer: &mut (impl AsyncWriteExt + Unpin),
    value: &T,
) -> Result<()> {
    writer
        .write_all(serde_json::to_string(value)?.as_bytes())
        .await?;
    writer.write_all(b"\n").await?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::{fs, os::unix::fs::PermissionsExt, path::PathBuf};

    use tokio::{
        io::{AsyncBufReadExt, AsyncWriteExt, BufReader},
        net::UnixStream,
        sync::{broadcast, mpsc},
    };

    use super::{start_server, ControlRequest, ControlResponse};

    #[test]
    fn control_request_is_stable_json() {
        let request = ControlRequest::SetComponents {
            components: vec!["vision".to_string(), "behavior".to_string()],
        };
        assert_eq!(
            serde_json::to_string(&request).unwrap(),
            r#"{"command":"set-components","components":["vision","behavior"]}"#
        );
        assert_eq!(
            serde_json::to_string(&ControlRequest::SetConfig {
                fieldname: "labor".to_string(),
                game_settings: Some("/tmp/settings.yaml".to_string()),
            })
            .unwrap(),
            r#"{"command":"set-config","fieldname":"labor","game_settings":"/tmp/settings.yaml"}"#
        );
    }

    #[tokio::test]
    async fn control_socket_is_private_and_round_trips() {
        let path = PathBuf::from(format!(
            "/tmp/teamplayer-control-test-{}.sock",
            std::process::id()
        ));
        let (command_tx, mut command_rx) = mpsc::unbounded_channel();
        let (event_tx, _) = broadcast::channel(4);
        let server = start_server(path.clone(), command_tx, event_tx)
            .await
            .unwrap();
        assert_eq!(
            fs::metadata(&path).unwrap().permissions().mode() & 0o777,
            0o600
        );

        let responder = tokio::spawn(async move {
            let command = command_rx.recv().await.unwrap();
            command
                .reply
                .send(ControlResponse::Ok {
                    message: "ready".to_string(),
                })
                .unwrap();
        });
        let stream = UnixStream::connect(&path).await.unwrap();
        let (reader, mut writer) = stream.into_split();
        writer
            .write_all(b"{\"command\":\"status\"}\n")
            .await
            .unwrap();
        let mut lines = BufReader::new(reader).lines();
        assert_eq!(
            lines.next_line().await.unwrap().unwrap(),
            "{\"type\":\"ok\",\"message\":\"ready\"}"
        );

        responder.await.unwrap();
        server.abort();
        fs::remove_file(path).unwrap();
    }
}
