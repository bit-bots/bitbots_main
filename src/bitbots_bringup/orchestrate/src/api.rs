//! Types exchanged over the HTTP API and the location of the API socket.

use std::path::{Path, PathBuf};

use anyhow::{bail, Context, Result};
use serde::{Deserialize, Serialize};

pub const API_PREFIX: &str = "/api/v1";
pub const DEFAULT_INSTANCE_NAME: &str = "default";

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProcState {
    Idle,
    Running,
    /// SIGTERM sent, waiting for exit
    Stopping,
    /// SIGTERM sent, will restart automatically when all processes have exited
    Restarting,
    Stopped,
    Crashed,
}

impl ProcState {
    pub fn label(&self) -> &'static str {
        match self {
            ProcState::Idle => "IDLE",
            ProcState::Running => "RUN ",
            ProcState::Stopping => "STOP",
            ProcState::Restarting => "RSTR",
            ProcState::Stopped => "DONE",
            ProcState::Crashed => "FAIL",
        }
    }

    /// Whether processes of the component may still be alive.
    pub fn is_alive(&self) -> bool {
        matches!(
            self,
            ProcState::Running | ProcState::Stopping | ProcState::Restarting
        )
    }
}

/// Parameters passed to the components when they are started.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Config {
    pub sim: bool,
    pub fieldname: String,
    pub dsd_file: String,
}

/// Partial update of the [`Config`]; fields that are `None` stay unchanged.
///
/// Switching `sim` also switches the simulator and hardware-only components and resets the
/// fieldname to the default of the new mode, unless a fieldname is given in the same update.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct ConfigUpdate {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sim: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fieldname: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dsd_file: Option<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ComponentInfo {
    pub key: String,
    pub name: String,
    pub enabled: bool,
    pub state: ProcState,
    pub pids: Vec<u32>,
    pub exit_code: Option<i32>,
    /// Always started, not toggleable in the frontend
    pub infrastructure: bool,
    /// Only valid on real hardware
    pub hardware_only: bool,
    /// Only used in simulation (e.g. the simulator itself)
    pub sim_component: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Status {
    pub config: Config,
    /// All components, always in the same order
    pub components: Vec<ComponentInfo>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SetEnabled {
    pub enabled: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LogStream {
    Stdout,
    Stderr,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LogLine {
    /// Strictly increasing over all components of one orchestrate instance
    pub seq: u64,
    pub component: String,
    pub component_name: String,
    pub stream: LogStream,
    pub text: String,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct LogsQuery {
    /// Only return lines with a larger sequence number
    #[serde(default)]
    pub since: u64,
    /// Only return lines of this component
    pub component: Option<String>,
    /// Maximum number of lines, the oldest matching lines are returned first
    pub limit: Option<usize>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Logs {
    pub lines: Vec<LogLine>,
    /// Lines after `since` were dropped from the buffer before they could be returned
    pub truncated: bool,
    /// Sequence number of the newest line of this instance (0 if there is none)
    pub latest_seq: u64,
    /// Changes when the orchestrator is restarted, which also restarts the sequence numbers
    pub instance_id: u64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApiError {
    pub error: String,
}

/// Directory holding the sockets and lock files of all local orchestrate instances.
pub fn default_socket_dir() -> PathBuf {
    match std::env::var_os("XDG_RUNTIME_DIR") {
        Some(dir) if !dir.is_empty() => PathBuf::from(dir).join("bitbots_orchestrate"),
        _ => {
            let dir =
                std::env::temp_dir().join(format!("bitbots_orchestrate-{}", nix::unistd::getuid()));
            eprintln!(
                "warning: XDG_RUNTIME_DIR is not set, using {} for the orchestrate socket",
                dir.display()
            );
            dir
        }
    }
}

/// Socket of the instance `name`, unless an explicit socket path is given.
pub fn resolve_socket(socket: Option<PathBuf>, name: &str) -> Result<PathBuf> {
    if let Some(socket) = socket {
        return Ok(socket);
    }
    if name.is_empty()
        || !name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
    {
        bail!("invalid instance name '{name}', use only letters, digits, '_' and '-'");
    }
    Ok(default_socket_dir().join(format!("{name}.sock")))
}

/// Lock file guarding the socket against a second orchestrate instance.
pub fn lock_path(socket: &Path) -> Result<PathBuf> {
    let file_name = socket
        .file_name()
        .with_context(|| format!("socket path {} has no file name", socket.display()))?;
    let mut lock_name = file_name.to_os_string();
    lock_name.push(".lock");
    Ok(socket.with_file_name(lock_name))
}
