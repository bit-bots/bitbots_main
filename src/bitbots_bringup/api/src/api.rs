//! Types exchanged over the HTTP API and the location of the API socket.

use std::path::{Path, PathBuf};

use anyhow::{bail, Context, Result};
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

pub use bitbots_stack::{ParamType, Value};

pub const API_PREFIX: &str = "/api/v1";
pub const DEFAULT_INSTANCE_NAME: &str = "default";

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProcState {
    Idle,
    /// Start requested, waiting for required components to run
    Waiting,
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
            ProcState::Waiting => "WAIT",
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

    /// Whether the component is alive or about to start.
    pub fn is_started(&self) -> bool {
        self.is_alive() || *self == ProcState::Waiting
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ParameterInfo {
    pub name: String,
    #[serde(rename = "type")]
    pub param_type: ParamType,
    pub description: String,
    pub value: Value,
    /// The only valid values, if they are restricted
    pub choices: Option<Vec<String>>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ProfileInfo {
    pub name: String,
    pub description: Option<String>,
}

/// New values for some parameters, by name. They take effect on the next start of a component.
pub type SetParameters = IndexMap<String, Value>;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ComponentInfo {
    pub key: String,
    pub name: String,
    pub description: Option<String>,
    /// Selected by the user, profile or default; ignored for locked components and fallbacks
    pub enabled: bool,
    /// Always runs while available
    pub locked: bool,
    /// Runs exactly when this component does not
    pub fallback_for: Option<String>,
    pub requires: Vec<String>,
    /// The parameter conditions of the component are met
    pub available: bool,
    /// Available and selected, i.e. started by start_all
    pub active: bool,
    pub state: ProcState,
    pub pids: Vec<u32>,
    pub exit_code: Option<i32>,
}

impl ComponentInfo {
    /// Whether users may switch the component on and off.
    pub fn toggleable(&self) -> bool {
        self.available && !self.locked && self.fallback_for.is_none()
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Status {
    /// In declaration order
    pub parameters: Vec<ParameterInfo>,
    pub profiles: Vec<ProfileInfo>,
    /// The profile applied last, if any
    pub profile: Option<String>,
    /// Inconsistencies that prevent start_all, e.g. a missing requirement
    pub problems: Vec<String>,
    /// All components, always in the same order
    pub components: Vec<ComponentInfo>,
    /// State of the global parameter blackboard, if the stack declares one
    pub blackboard: Option<BlackboardSummary>,
}

/// Value of a global parameter, with the types of ROS parameters.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum GlobalValue {
    Bool(bool),
    Integer(i64),
    Double(f64),
    String(String),
    BoolArray(Vec<bool>),
    IntegerArray(Vec<i64>),
    DoubleArray(Vec<f64>),
    StringArray(Vec<String>),
}

impl GlobalValue {
    pub fn type_name(&self) -> &'static str {
        match self {
            GlobalValue::Bool(_) => "bool",
            GlobalValue::Integer(_) => "integer",
            GlobalValue::Double(_) => "double",
            GlobalValue::String(_) => "string",
            GlobalValue::BoolArray(_) => "bool array",
            GlobalValue::IntegerArray(_) => "integer array",
            GlobalValue::DoubleArray(_) => "double array",
            GlobalValue::StringArray(_) => "string array",
        }
    }
}

impl std::fmt::Display for GlobalValue {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GlobalValue::Bool(v) => write!(f, "{v}"),
            GlobalValue::Integer(v) => write!(f, "{v}"),
            GlobalValue::Double(v) => write!(f, "{v}"),
            GlobalValue::String(v) => f.write_str(v),
            GlobalValue::BoolArray(v) => write!(f, "{v:?}"),
            GlobalValue::IntegerArray(v) => write!(f, "{v:?}"),
            GlobalValue::DoubleArray(v) => write!(f, "{v:?}"),
            GlobalValue::StringArray(v) => write!(f, "{v:?}"),
        }
    }
}

/// Where the value of a global parameter comes from.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GlobalSource {
    /// A parameter file, given as path
    File(String),
    /// Derived from a stack parameter
    Stack,
    /// An override set through the API or the command line
    Override,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", tag = "state", content = "detail")]
pub enum RosState {
    /// The ROS node waits for these components to run
    WaitingFor(Vec<String>),
    Starting,
    Running,
    /// Setting up the ROS node failed, it is retried
    Failed(String),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BlackboardSummary {
    pub node: String,
    pub ros: RosState,
    /// Why the global parameters could not be assembled; nothing new is published then
    pub error: Option<String>,
    pub overrides: IndexMap<String, GlobalValue>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GlobalParameterInfo {
    pub name: String,
    pub value: GlobalValue,
    pub source: GlobalSource,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BlackboardInfo {
    pub summary: BlackboardSummary,
    /// The published parameters, sorted by name; empty while `summary.error` is set
    pub parameters: Vec<GlobalParameterInfo>,
    /// Topics of the published field maps
    pub maps: Vec<String>,
}

/// Overrides of global parameters by name; `null` removes an override.
pub type SetOverrides = IndexMap<String, Option<GlobalValue>>;

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

/// Pid of the orchestrate instance holding the lock of `socket`, as recorded in the lock file.
///
/// Only meaningful while an instance is listening; the file outlives its instance.
pub fn lock_owner(socket: &Path) -> Option<u32> {
    let text = std::fs::read_to_string(lock_path(socket).ok()?).ok()?;
    text.trim().parse().ok()
}
