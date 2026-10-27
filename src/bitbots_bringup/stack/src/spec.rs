//! Serde model of the stack file, as written by humans.

use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ParamType {
    Bool,
    String,
}

/// A parameter value; its type has to match the declared [`ParamType`].
#[derive(Clone, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(untagged)]
pub enum Value {
    Bool(bool),
    String(String),
}

impl Value {
    pub fn param_type(&self) -> ParamType {
        match self {
            Value::Bool(_) => ParamType::Bool,
            Value::String(_) => ParamType::String,
        }
    }
}

impl std::fmt::Display for Value {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Value::Bool(b) => write!(f, "{b}"),
            Value::String(s) => f.write_str(s),
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct StackFile {
    pub version: u32,
    #[serde(default)]
    pub parameters: IndexMap<String, ParameterSpec>,
    #[serde(default)]
    pub profiles: IndexMap<String, ProfileSpec>,
    pub components: IndexMap<String, ComponentSpec>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct ParameterSpec {
    #[serde(rename = "type")]
    pub param_type: ParamType,
    pub default: Value,
    pub description: String,
}

#[derive(Debug, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct ProfileSpec {
    #[serde(default)]
    pub description: Option<String>,
    #[serde(default)]
    pub parameters: IndexMap<String, Value>,
    #[serde(default)]
    pub enable: Vec<String>,
    #[serde(default)]
    pub disable: Vec<String>,
}

fn default_true() -> bool {
    true
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct ComponentSpec {
    pub name: String,
    #[serde(default)]
    pub description: Option<String>,
    /// Default selection, overridden by profiles and users
    #[serde(default = "default_true")]
    pub enabled: bool,
    /// Always runs while available, cannot be disabled
    #[serde(default)]
    pub locked: bool,
    /// Parameter values that all have to match for the component to be available
    #[serde(default, rename = "if")]
    pub condition: IndexMap<String, Value>,
    /// Components that have to run before this one starts
    #[serde(default)]
    pub requires: Vec<String>,
    /// Runs exactly when the named component does not
    #[serde(default)]
    pub fallback_for: Option<String>,
    // A single process can be given directly, several ones under `processes`.
    #[serde(default)]
    pub launch: Option<LaunchSpec>,
    #[serde(default)]
    pub run: Option<RunSpec>,
    #[serde(default)]
    pub command: Option<Vec<String>>,
    // Entries are written as `- launch: {...}` rather than with YAML tags.
    #[serde(default, with = "serde_norway::with::singleton_map_recursive")]
    pub processes: Option<Vec<ProcessSpec>>,
}

/// One process of a component.
#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum ProcessSpec {
    /// `ros2 launch <package> <file> <arg>:=<value>...`
    Launch(LaunchSpec),
    /// `ros2 run <package> <executable> <args>...`
    Run(RunSpec),
    /// Any other command, given as program and arguments
    Command(Vec<String>),
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LaunchSpec {
    pub package: String,
    pub file: String,
    #[serde(default)]
    pub args: IndexMap<String, String>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunSpec {
    pub package: String,
    pub executable: String,
    #[serde(default)]
    pub args: Vec<String>,
}
