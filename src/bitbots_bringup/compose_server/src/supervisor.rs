//! Starts, stops and restarts the components and keeps track of their processes.

use std::{collections::HashMap, path::PathBuf, sync::Arc};

use anyhow::{bail, Context};
use bitbots_compose_file::{Resolved, Stack, Value, Values};
use indexmap::IndexMap;
use tokio::{
    io::{AsyncBufReadExt, BufReader},
    process::Command,
    sync::mpsc,
};

use crate::{
    api::{
        ComponentInfo, GlobalValue, LogStream, ParameterInfo, ProcState, ProfileInfo,
        SetGlobalParameters, SetParameters, Status,
    },
    logs::LogStore,
    share::{self, ShareResolver},
};

/// Runtime state of a component.
pub struct ComponentState {
    pub state: ProcState,
    /// Process group ids of the processes of the current run that have not exited yet.
    pub pids: Vec<u32>,
    /// First exit code reported in the current run; decides between Stopped and
    /// Crashed once all processes of the component have exited.
    pub exit_code: Option<i32>,
}

enum SupervisorMsg {
    ProcessExited {
        comp_idx: usize,
        pid: u32,
        code: i32,
    },
}

/// Why an action on a component was refused.
#[derive(Debug, PartialEq)]
pub enum ActionError {
    /// The action does not fit the current state of the component
    Conflict(String),
    /// The request itself is invalid
    BadRequest(String),
    /// The compose server could not carry out a valid request
    Failed(String),
}

impl std::fmt::Display for ActionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ActionError::Conflict(msg)
            | ActionError::BadRequest(msg)
            | ActionError::Failed(msg) => f.write_str(msg),
        }
    }
}

/// Writes the output of one component to the [`LogStore`].
#[derive(Clone)]
struct ComponentLog {
    key: Arc<str>,
    name: Arc<str>,
    logs: Arc<LogStore>,
}

impl ComponentLog {
    fn push(&self, stream: LogStream, text: &str) {
        self.logs.push(&self.key, &self.name, stream, text);
    }

    /// Messages from the compose server itself, as opposed to output of the child processes.
    fn push_compose(&self, msg: &str) {
        self.push(LogStream::Stderr, &format!("[compose] {msg}"));
    }

    async fn pump<R: tokio::io::AsyncRead + Unpin>(&self, mut r: BufReader<R>, stream: LogStream) {
        let mut line = String::new();
        while let Ok(n) = r.read_line(&mut line).await {
            if n == 0 {
                break;
            }
            self.push(stream, line.trim_end_matches('\n'));
            line.clear();
        }
    }
}

pub struct Supervisor {
    stack: Stack,
    values: Values,
    /// Selection of every component, in stack order
    enabled: Vec<bool>,
    /// The profile applied last
    profile: Option<String>,
    /// Valid values of the parameters that declare `choices_from`
    choices: HashMap<String, Vec<String>>,
    /// Overrides of global parameters, written to `global_parameters_file`
    global_parameters: IndexMap<String, GlobalValue>,
    global_parameters_file: PathBuf,
    /// Runtime state of every component, in stack order
    pub components: Vec<ComponentState>,
    /// Set once shutdown has begun; no components are started afterwards.
    shutting_down: bool,
    logs: Arc<LogStore>,
    msg_tx: mpsc::UnboundedSender<SupervisorMsg>,
    msg_rx: mpsc::UnboundedReceiver<SupervisorMsg>,
}

impl Supervisor {
    /// `global_parameters_file` is where the overrides of global parameters are written for the
    /// components. Fails if the choices of a parameter cannot be found or the file cannot be
    /// written.
    pub fn new(
        stack: Stack,
        logs: Arc<LogStore>,
        share: ShareResolver,
        global_parameters_file: PathBuf,
    ) -> anyhow::Result<Self> {
        let mut choices = HashMap::new();
        for p in stack.parameters() {
            if let Some(from) = &p.choices_from {
                let dir = share(&from.package)?.join(&from.directory);
                let names = share::subdirectories(&dir)
                    .with_context(|| format!("choices of parameter '{}'", p.name))?;
                if let Value::String(default) = &p.default {
                    if !names.contains(default) {
                        bail!(
                            "default '{default}' of parameter '{}' is none of its choices: {}",
                            p.name,
                            names.join(", ")
                        );
                    }
                }
                choices.insert(p.name.clone(), names);
            }
        }
        // Components always receive a valid file, also without overrides. This also replaces
        // the file of an earlier run of this instance.
        write_parameter_file(&global_parameters_file, &IndexMap::new())
            .context("cannot write the global parameters")?;
        let (msg_tx, msg_rx) = mpsc::unbounded_channel();
        let components = stack
            .components()
            .iter()
            .map(|_| ComponentState {
                state: ProcState::Idle,
                pids: Vec::new(),
                exit_code: None,
            })
            .collect();
        let sup = Self {
            values: stack.default_values(),
            enabled: stack.default_enabled(),
            profile: None,
            choices,
            global_parameters: IndexMap::new(),
            global_parameters_file,
            stack,
            components,
            shutting_down: false,
            logs,
            msg_tx,
            msg_rx,
        };
        Ok(sup)
    }

    /// Set or remove overrides of global parameters, all or none. They are passed to the
    /// components on their next start; running components keep the values they started with.
    pub fn set_global_parameters(
        &mut self,
        changes: SetGlobalParameters,
    ) -> Result<(), ActionError> {
        let mut global_parameters = self.global_parameters.clone();
        for (name, value) in changes {
            let valid = !name.is_empty()
                && name.split('.').all(|part| {
                    !part.is_empty() && part.chars().all(|c| c.is_ascii_alphanumeric() || c == '_')
                });
            if !valid {
                return Err(ActionError::BadRequest(format!(
                    "'{name}' is no valid ROS parameter name"
                )));
            }
            match value {
                Some(value) => global_parameters.insert(name, value),
                None => global_parameters.shift_remove(&name),
            };
        }
        write_parameter_file(&self.global_parameters_file, &global_parameters).map_err(|e| {
            ActionError::Failed(format!("cannot write the global parameters: {e:#}"))
        })?;
        self.global_parameters = global_parameters;
        Ok(())
    }

    /// Path substituted for `${global_parameters_file}`; the file always exists and holds no
    /// parameters while there are no overrides.
    fn global_parameters_path(&self) -> String {
        self.global_parameters_file.display().to_string()
    }

    /// The commands of a component, refused if a launch argument would be empty, which
    /// `ros2 launch` rejects.
    fn component_commands(&self, comp_idx: usize) -> Result<Vec<Vec<String>>, ActionError> {
        let cmds = self
            .stack
            .commands(comp_idx, &self.values, &self.global_parameters_path());
        for cmd in &cmds {
            if cmd.get(1).map(String::as_str) != Some("launch") {
                continue;
            }
            if let Some(arg) = cmd.iter().skip(4).find(|a| a.ends_with(":=")) {
                return Err(ActionError::Conflict(format!(
                    "{}: launch argument '{arg}' would be empty, which ros2 launch rejects",
                    self.stack.components()[comp_idx].key
                )));
            }
        }
        Ok(cmds)
    }

    /// Fails if a value is none of the parameter's choices.
    fn check_choices(&self, values: &Values) -> Result<(), ActionError> {
        for (name, value) in values {
            if let (Some(choices), Value::String(v)) = (self.choices.get(name), value) {
                if !choices.contains(v) {
                    return Err(ActionError::BadRequest(format!(
                        "'{v}' is no valid {name}, choose one of: {}",
                        choices.join(", ")
                    )));
                }
            }
        }
        Ok(())
    }

    pub fn stack(&self) -> &Stack {
        &self.stack
    }

    /// Apply the command line selection: a profile, then single parameters (`name=value`),
    /// then component switches.
    pub fn apply_cli(
        &mut self,
        profile: Option<&str>,
        set: &[String],
        enable: &[String],
        disable: &[String],
        globals: &[String],
    ) -> anyhow::Result<()> {
        if let Some(profile) = profile {
            self.apply_profile(profile)
                .map_err(|e| anyhow::anyhow!("{e}"))?;
        }
        let mut params = SetParameters::new();
        for assignment in set {
            let Some((name, raw)) = assignment.split_once('=') else {
                bail!("expected name=value, got '{assignment}'");
            };
            params.insert(name.to_string(), self.stack.parse_value(name, raw)?);
        }
        self.set_parameters(params)
            .map_err(|e| anyhow::anyhow!("{e}"))?;
        for (keys, enabled) in [(enable, true), (disable, false)] {
            for key in keys {
                let Some(idx) = self.stack.component_idx(key) else {
                    let available: Vec<&str> = self
                        .stack
                        .components()
                        .iter()
                        .map(|c| c.key.as_str())
                        .collect();
                    bail!(
                        "unknown component key '{key}', available keys: {}",
                        available.join(", ")
                    );
                };
                self.set_enabled(idx, enabled)
                    .map_err(|e| anyhow::anyhow!("{e}"))?;
            }
        }
        let mut changes = SetGlobalParameters::new();
        for assignment in globals {
            let Some((name, raw)) = assignment.split_once('=') else {
                bail!("expected name=value, got '{assignment}'");
            };
            changes.insert(name.to_string(), Some(parse_global_value(name, raw)?));
        }
        self.set_global_parameters(changes)
            .map_err(|e| anyhow::anyhow!("{e}"))?;
        Ok(())
    }

    /// Reset parameters and selection to the defaults plus the profile, so profiles do not
    /// accumulate. Takes effect on the next start of the affected components.
    pub fn apply_profile(&mut self, name: &str) -> Result<(), ActionError> {
        let mut values = self.stack.default_values();
        let mut enabled = self.stack.default_enabled();
        self.stack
            .apply_profile(name, &mut values, &mut enabled)
            .map_err(|e| ActionError::BadRequest(e.to_string()))?;
        self.check_choices(&values)?;
        self.values = values;
        self.enabled = enabled;
        self.profile = Some(name.to_string());
        Ok(())
    }

    /// Set all given parameters, or none if one of them is invalid. Takes effect on the next
    /// start of the affected components.
    pub fn set_parameters(&mut self, params: SetParameters) -> Result<(), ActionError> {
        for (name, value) in &params {
            self.stack
                .check_value(name, value)
                .map_err(|e| ActionError::BadRequest(e.to_string()))?;
        }
        self.check_choices(&params)?;
        self.values.extend(params);
        Ok(())
    }

    pub fn set_enabled(&mut self, comp_idx: usize, enabled: bool) -> Result<(), ActionError> {
        let c = &self.stack.components()[comp_idx];
        if !c.toggleable() {
            return Err(ActionError::Conflict(format!(
                "{} is locked or a fallback and cannot be switched",
                c.key
            )));
        }
        self.enabled[comp_idx] = enabled;
        Ok(())
    }

    fn resolved(&self) -> Resolved {
        self.stack.resolve(&self.values, &self.enabled)
    }

    pub fn status(&self) -> Status {
        let resolved = self.resolved();
        Status {
            parameters: self
                .stack
                .parameters()
                .iter()
                .map(|p| ParameterInfo {
                    name: p.name.clone(),
                    param_type: p.param_type,
                    description: p.description.clone(),
                    value: self.values[&p.name].clone(),
                    choices: self.choices.get(&p.name).cloned(),
                })
                .collect(),
            profiles: self
                .stack
                .profiles()
                .iter()
                .map(|p| ProfileInfo {
                    name: p.name.clone(),
                    description: p.description.clone(),
                })
                .collect(),
            profile: self.profile.clone(),
            problems: resolved.problems,
            components: self
                .stack
                .components()
                .iter()
                .zip(&self.components)
                .zip(&resolved.components)
                .zip(&self.enabled)
                .map(|(((def, comp), res), &enabled)| ComponentInfo {
                    key: def.key.clone(),
                    name: def.name.clone(),
                    description: def.description.clone(),
                    enabled,
                    locked: def.locked,
                    fallback_for: def.fallback_for.clone(),
                    requires: def.requires.clone(),
                    available: res.available,
                    active: res.active,
                    state: comp.state,
                    pids: comp.pids.clone(),
                    exit_code: comp.exit_code,
                })
                .collect(),
            global_parameters: self.global_parameters.clone(),
        }
    }

    pub fn any_alive(&self) -> bool {
        self.components.iter().any(|c| c.state.is_alive())
    }

    fn component_log(&self, comp_idx: usize) -> ComponentLog {
        let def = &self.stack.components()[comp_idx];
        ComponentLog {
            key: def.key.as_str().into(),
            name: def.name.as_str().into(),
            logs: self.logs.clone(),
        }
    }

    /// Requirements of a component that are not running yet.
    fn missing_requirements(&self, comp_idx: usize) -> Vec<&str> {
        self.stack.components()[comp_idx]
            .requires
            .iter()
            .filter(|req| {
                let r = self.stack.component_idx(req).expect("validated on load");
                self.components[r].state != ProcState::Running
            })
            .map(String::as_str)
            .collect()
    }

    /// Start a component, or let it wait until its requirements run.
    pub fn start_component(&mut self, comp_idx: usize) -> Result<(), ActionError> {
        let def = &self.stack.components()[comp_idx];
        if self.shutting_down {
            return Err(ActionError::Conflict(
                "the compose server is shutting down".to_string(),
            ));
        }
        if self.components[comp_idx].state.is_started() {
            return Err(ActionError::Conflict(format!(
                "{} is already running",
                def.key
            )));
        }
        let resolved = self.resolved();
        if !resolved.components[comp_idx].available {
            return Err(ActionError::Conflict(format!(
                "{} is not available with the current parameters",
                def.key
            )));
        }
        for req in &def.requires {
            let r = self.stack.component_idx(req).expect("validated on load");
            if !resolved.components[r].active {
                return Err(ActionError::Conflict(format!(
                    "{} requires {req}, which is not selected or not available",
                    def.key
                )));
            }
        }
        // Fail now rather than after waiting for the requirements.
        let cmds = self.component_commands(comp_idx)?;
        let missing = self.missing_requirements(comp_idx);
        if missing.is_empty() {
            self.spawn_commands(comp_idx, cmds);
        } else {
            let msg = format!("waiting for {} to run", missing.join(", "));
            self.component_log(comp_idx).push_compose(&msg);
            self.components[comp_idx].state = ProcState::Waiting;
        }
        Ok(())
    }

    /// Spawn all commands of a component, each in its own process group.
    ///
    /// If one command fails to spawn, the already spawned ones are terminated and the
    /// component ends up as Crashed once they have exited.
    fn spawn_commands(&mut self, comp_idx: usize, cmds: Vec<Vec<String>>) {
        // Starting over while processes of the previous run are alive would orphan them.
        assert!(
            self.components[comp_idx].pids.is_empty(),
            "start of {} while processes of the previous run are still alive",
            self.stack.components()[comp_idx].key
        );
        let log = self.component_log(comp_idx);
        let msg_tx = self.msg_tx.clone();
        let comp = &mut self.components[comp_idx];
        comp.state = ProcState::Running;
        comp.exit_code = None;

        for cmd_args in cmds {
            if cmd_args.is_empty() {
                continue;
            }
            let mut child = match Command::new(&cmd_args[0])
                .args(&cmd_args[1..])
                .stdout(std::process::Stdio::piped())
                .stderr(std::process::Stdio::piped())
                .process_group(0)
                .kill_on_drop(true)
                .spawn()
            {
                Ok(c) => c,
                Err(e) => {
                    log.push_compose(&format!("failed to spawn `{}`: {e}", cmd_args.join(" ")));
                    comp.exit_code = Some(-1);
                    if comp.pids.is_empty() {
                        comp.state = ProcState::Crashed;
                    } else {
                        log.push_compose("stopping the already started processes");
                        signal_pids(&comp.pids, nix::sys::signal::Signal::SIGTERM);
                        comp.state = ProcState::Stopping;
                    }
                    return;
                }
            };

            let pid = child
                .id()
                .expect("a freshly spawned child that was never awaited has a pid");
            comp.pids.push(pid);

            let stdout = child.stdout.take().map(BufReader::new);
            let stderr = child.stderr.take().map(BufReader::new);
            let tx = msg_tx.clone();
            let log = log.clone();

            tokio::spawn(async move {
                // Read stdout and stderr concurrently so neither blocks the other.
                tokio::join!(
                    async {
                        if let Some(r) = stdout {
                            log.pump(r, LogStream::Stdout).await;
                        }
                    },
                    async {
                        if let Some(r) = stderr {
                            log.pump(r, LogStream::Stderr).await;
                        }
                    }
                );
                let code = child
                    .wait()
                    .await
                    .map(|s| s.code().unwrap_or(-1))
                    .unwrap_or(-1);
                let _ = tx.send(SupervisorMsg::ProcessExited {
                    comp_idx,
                    pid,
                    code,
                });
            });
        }
    }

    /// Send SIGTERM to the component's processes and mark it as Stopping.
    pub fn stop_component(&mut self, comp_idx: usize) {
        let comp = &mut self.components[comp_idx];
        match comp.state {
            ProcState::Running => {
                signal_pids(&comp.pids, nix::sys::signal::Signal::SIGTERM);
                comp.state = ProcState::Stopping;
            }
            ProcState::Restarting => {
                // Already sent SIGTERM — cancel the pending restart
                comp.state = ProcState::Stopping;
            }
            ProcState::Waiting => comp.state = ProcState::Idle,
            _ => {}
        }
    }

    /// Restart a running component once all of its processes have exited, or start a
    /// component that is not running.
    pub fn restart_component(&mut self, comp_idx: usize) -> Result<(), ActionError> {
        let comp = &mut self.components[comp_idx];
        match comp.state {
            ProcState::Running => {
                signal_pids(&comp.pids, nix::sys::signal::Signal::SIGTERM);
                comp.state = ProcState::Restarting;
                Ok(())
            }
            ProcState::Stopping | ProcState::Restarting => Err(ActionError::Conflict(format!(
                "{} is still stopping",
                self.stack.components()[comp_idx].key
            ))),
            _ => self.start_component(comp_idx),
        }
    }

    pub fn stop_all(&mut self) {
        for i in 0..self.components.len() {
            self.stop_component(i);
        }
    }

    /// Start all active components that are not running. Refused while the selection has
    /// problems, e.g. a missing requirement.
    pub fn start_all_enabled(&mut self) -> Result<(), ActionError> {
        let resolved = self.resolved();
        if !resolved.problems.is_empty() {
            return Err(ActionError::Conflict(resolved.problems.join("; ")));
        }
        let to_start: Vec<usize> = (0..self.components.len())
            .filter(|&i| resolved.components[i].active && !self.components[i].state.is_started())
            .collect();
        for i in to_start {
            self.start_component(i)?;
        }
        Ok(())
    }

    /// Stop everything and refuse to start components from now on.
    pub fn begin_shutdown(&mut self) {
        self.shutting_down = true;
        self.stop_all();
    }

    /// Send SIGKILL to all processes that are still alive.
    pub fn kill_remaining(&mut self) {
        for (comp, def) in self.components.iter().zip(self.stack.components()) {
            if !comp.pids.is_empty() {
                self.logs.push(
                    &def.key,
                    &def.name,
                    LogStream::Stderr,
                    &format!(
                        "[compose] processes {:?} did not exit after SIGTERM, sending SIGKILL",
                        comp.pids
                    ),
                );
                signal_pids(&comp.pids, nix::sys::signal::Signal::SIGKILL);
            }
        }
    }

    /// Process exit reports and restart components whose restart is pending.
    ///
    /// A component only leaves Running/Stopping/Restarting once all of its processes have
    /// exited, so a multi-process component is never restarted while parts of it still run.
    pub fn tick(&mut self) {
        for comp_idx in self.drain_messages() {
            if let Err(e) = self.start_component(comp_idx) {
                self.component_log(comp_idx)
                    .push_compose(&format!("restart failed: {e}"));
            }
        }
        for comp_idx in 0..self.components.len() {
            if self.components[comp_idx].state == ProcState::Waiting
                && self.missing_requirements(comp_idx).is_empty()
            {
                // The parameters may have changed while it waited.
                match self.component_commands(comp_idx) {
                    Ok(cmds) => self.spawn_commands(comp_idx, cmds),
                    Err(e) => {
                        self.component_log(comp_idx)
                            .push_compose(&format!("cannot start: {e}"));
                        self.components[comp_idx].state = ProcState::Crashed;
                    }
                }
            }
        }
    }

    /// Process exit reports and return the components whose restart is pending.
    fn drain_messages(&mut self) -> Vec<usize> {
        let mut to_restart = Vec::new();
        while let Ok(msg) = self.msg_rx.try_recv() {
            match msg {
                SupervisorMsg::ProcessExited {
                    comp_idx,
                    pid,
                    code,
                } => {
                    let log = self.component_log(comp_idx);
                    let comp = &mut self.components[comp_idx];
                    let Some(pos) = comp.pids.iter().position(|&p| p == pid) else {
                        log.push_compose(&format!(
                            "exit of untracked pid {pid} (code {code}) reported, ignoring it"
                        ));
                        continue;
                    };
                    comp.pids.swap_remove(pos);
                    comp.exit_code.get_or_insert(code);

                    if comp.state == ProcState::Running {
                        log.push_compose(&format!(
                            "process {pid} exited unexpectedly with code {code}"
                        ));
                        if !comp.pids.is_empty() {
                            // Do not leave the component half running.
                            log.push_compose("stopping the remaining processes");
                            signal_pids(&comp.pids, nix::sys::signal::Signal::SIGTERM);
                            comp.state = ProcState::Stopping;
                        }
                    }

                    if !comp.pids.is_empty() {
                        continue;
                    }
                    match comp.state {
                        ProcState::Restarting if !self.shutting_down => {
                            comp.state = ProcState::Idle;
                            to_restart.push(comp_idx);
                        }
                        ProcState::Restarting | ProcState::Stopping | ProcState::Running => {
                            comp.state = if comp.exit_code == Some(0) {
                                ProcState::Stopped
                            } else {
                                ProcState::Crashed
                            };
                        }
                        _ => {}
                    }
                }
            }
        }
        to_restart
    }
}

/// Parse a global parameter value given as text, e.g. on the command line. The text is read as
/// YAML, so `2` is an integer, `2.0` a double, `true` a bool and `[1, 2]` an integer array;
/// quote a value to pass it as string.
pub fn parse_global_value(name: &str, raw: &str) -> anyhow::Result<GlobalValue> {
    let value: GlobalValue = serde_norway::from_str(raw)
        .with_context(|| format!("'{raw}' is no valid value for global parameter '{name}'"))?;
    Ok(value)
}

/// Write overrides as ROS parameter file for any node, nesting the dotted names.
fn write_parameter_file(
    path: &std::path::Path,
    parameters: &IndexMap<String, GlobalValue>,
) -> anyhow::Result<()> {
    use serde_norway::{Mapping, Value as Y};
    let mut root = Mapping::new();
    for (name, value) in parameters {
        let mut map = &mut root;
        let mut parts: Vec<&str> = name.split('.').collect();
        let leaf = parts.pop().expect("validated as non-empty");
        for part in parts {
            let entry = map
                .entry(Y::String(part.to_string()))
                .or_insert_with(|| Y::Mapping(Mapping::new()));
            let Y::Mapping(next) = entry else {
                bail!("'{name}' nests below a parameter that has a value");
            };
            map = next;
        }
        if map.contains_key(leaf) {
            bail!("'{name}' is both a parameter and a prefix of other parameters");
        }
        map.insert(Y::String(leaf.to_string()), serde_norway::to_value(value)?);
    }
    let mut node = Mapping::new();
    node.insert(Y::String("ros__parameters".into()), Y::Mapping(root));
    let mut doc = Mapping::new();
    doc.insert(Y::String("/**".into()), Y::Mapping(node));
    let text = format!(
        "# Overrides of global parameters, written by bitbots_compose_server\n{}",
        serde_norway::to_string(&doc)?
    );
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    // Replace the file atomically, so that a starting component never reads half of it.
    let tmp = path.with_extension("tmp");
    std::fs::write(&tmp, text)?;
    std::fs::rename(&tmp, path)?;
    Ok(())
}

/// Signal the process groups led by the given pids.
fn signal_pids(pids: &[u32], signal: nix::sys::signal::Signal) {
    use nix::{sys::signal::killpg, unistd::Pid};
    for &pid in pids {
        let _ = killpg(Pid::from_raw(pid as i32), signal);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::Value;
    use std::time::Duration;

    /// Commands are replaced in the process tests, so they only need to be valid here.
    const TEST_STACK: &str = r#"
version: 1
parameters:
  sim: { type: bool, default: false, description: Simulation }
  field: { type: string, default: lab, description: Field }
profiles:
  sim:
    parameters: { sim: true, field: kid }
    disable: [extra]
components:
  base:
    name: Base
    command: [sleep, "30"]
  hardware:
    name: Hardware
    if: { sim: false }
    command: [sleep, "30"]
  pair:
    name: Pair
    requires: [base]
    processes:
      - command: [sleep, "30"]
      - command: [sleep, "30"]
  locked:
    name: Locked
    locked: true
    command: [sleep, "30"]
  extra:
    name: Extra
    command: [echo, "${field}"]
  board:
    name: Board
    command: [board, "file=${global_parameters_file}"]
  launched:
    name: Launched
    launch: { package: pkg, file: f.launch, args: { field: "${field}" } }
"#;

    fn cmd(args: &[&str]) -> Vec<String> {
        args.iter().map(|s| s.to_string()).collect()
    }

    fn no_share() -> ShareResolver {
        Box::new(|package| bail!("no share directory for {package} in this test"))
    }

    /// Path of the global parameter file of one test.
    fn global_file(test: &str) -> PathBuf {
        std::env::temp_dir()
            .join(format!("bitbots_compose_sup_{}", std::process::id()))
            .join(format!("{test}.global_parameters.yaml"))
    }

    fn supervisor() -> Supervisor {
        supervisor_for(&format!("{:?}", std::thread::current().id()))
    }

    fn supervisor_for(test: &str) -> Supervisor {
        Supervisor::new(
            Stack::parse(TEST_STACK).unwrap(),
            Arc::new(LogStore::new()),
            no_share(),
            global_file(test),
        )
        .unwrap()
    }

    fn idx(sup: &Supervisor, key: &str) -> usize {
        sup.stack.component_idx(key).unwrap()
    }

    /// Drain messages until the component settles in a non-transitional state.
    async fn wait_settled(sup: &mut Supervisor, comp_idx: usize) -> Vec<usize> {
        let mut to_restart = Vec::new();
        for _ in 0..200 {
            to_restart.extend(sup.drain_messages());
            if !to_restart.is_empty() || !sup.components[comp_idx].state.is_alive() {
                return to_restart;
            }
            tokio::time::sleep(Duration::from_millis(25)).await;
        }
        panic!(
            "component did not settle, state {:?}",
            sup.components[comp_idx].state
        );
    }

    #[tokio::test]
    async fn restart_waits_for_all_processes_of_a_component() {
        let mut sup = supervisor();
        let idx = idx(&sup, "pair");
        sup.spawn_commands(idx, vec![cmd(&["sleep", "30"]), cmd(&["sleep", "30"])]);
        assert_eq!(sup.components[idx].pids.len(), 2);

        sup.restart_component(idx).unwrap();
        let to_restart = wait_settled(&mut sup, idx).await;
        assert_eq!(to_restart, vec![idx]);
        assert!(sup.components[idx].pids.is_empty());

        // The new run must not be affected by exit reports of the old one.
        sup.spawn_commands(idx, vec![cmd(&["sleep", "30"]), cmd(&["sleep", "30"])]);
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert!(sup.drain_messages().is_empty());
        assert_eq!(sup.components[idx].state, ProcState::Running);
        assert_eq!(sup.components[idx].pids.len(), 2);

        sup.stop_component(idx);
        wait_settled(&mut sup, idx).await;
        assert!(sup.components[idx].pids.is_empty());
    }

    #[tokio::test]
    async fn partial_spawn_failure_stops_started_processes() {
        let mut sup = supervisor();
        let idx = idx(&sup, "pair");
        sup.spawn_commands(
            idx,
            vec![
                cmd(&["sleep", "30"]),
                cmd(&["/nonexistent/compose-test-binary"]),
            ],
        );
        assert_eq!(sup.components[idx].state, ProcState::Stopping);

        wait_settled(&mut sup, idx).await;
        assert_eq!(sup.components[idx].state, ProcState::Crashed);
        assert!(sup.components[idx].pids.is_empty());
    }

    #[tokio::test]
    async fn spawn_failure_of_only_process_crashes_immediately() {
        let mut sup = supervisor();
        let idx = idx(&sup, "base");
        sup.spawn_commands(idx, vec![cmd(&["/nonexistent/compose-test-binary"])]);
        assert_eq!(sup.components[idx].state, ProcState::Crashed);
        assert!(sup.components[idx].pids.is_empty());
    }

    #[tokio::test]
    async fn crash_of_one_process_stops_its_siblings() {
        let mut sup = supervisor();
        let idx = idx(&sup, "pair");
        sup.spawn_commands(
            idx,
            vec![cmd(&["sh", "-c", "exit 3"]), cmd(&["sleep", "30"])],
        );
        wait_settled(&mut sup, idx).await;
        assert_eq!(sup.components[idx].state, ProcState::Crashed);
        assert_eq!(sup.components[idx].exit_code, Some(3));
        assert!(sup.components[idx].pids.is_empty());
    }

    #[tokio::test]
    async fn clean_exit_is_reported_as_stopped() {
        let mut sup = supervisor();
        let idx = idx(&sup, "base");
        sup.spawn_commands(idx, vec![cmd(&["true"])]);
        wait_settled(&mut sup, idx).await;
        assert_eq!(sup.components[idx].state, ProcState::Stopped);
    }

    #[tokio::test]
    async fn output_and_spawn_errors_reach_the_log_store() {
        let logs = Arc::new(LogStore::new());
        let mut sup = Supervisor::new(
            Stack::parse(TEST_STACK).unwrap(),
            logs.clone(),
            no_share(),
            global_file("logs"),
        )
        .unwrap();
        let idx = idx(&sup, "base");
        sup.spawn_commands(idx, vec![cmd(&["sh", "-c", "echo out; echo err >&2"])]);
        wait_settled(&mut sup, idx).await;
        sup.spawn_commands(idx, vec![cmd(&["/nonexistent/compose-test-binary"])]);

        let lines = logs.since(0, Some("base"), usize::MAX).lines;
        let has = |stream, text: &str| {
            lines
                .iter()
                .any(|l| l.stream == stream && l.text.contains(text))
        };
        assert!(has(LogStream::Stdout, "out"));
        assert!(has(LogStream::Stderr, "err"));
        assert!(has(LogStream::Stderr, "failed to spawn"));
    }

    #[tokio::test]
    async fn components_wait_for_their_requirements() {
        let mut sup = supervisor();
        let (base, pair) = (idx(&sup, "base"), idx(&sup, "pair"));
        sup.start_component(pair).unwrap();
        assert_eq!(sup.components[pair].state, ProcState::Waiting);
        sup.tick();
        assert_eq!(sup.components[pair].state, ProcState::Waiting);

        sup.start_component(base).unwrap();
        sup.tick();
        assert_eq!(sup.components[pair].state, ProcState::Running);

        sup.begin_shutdown();
        wait_settled(&mut sup, pair).await;
        wait_settled(&mut sup, base).await;
    }

    #[tokio::test]
    async fn waiting_can_be_cancelled() {
        let mut sup = supervisor();
        let pair = idx(&sup, "pair");
        sup.start_component(pair).unwrap();
        sup.stop_component(pair);
        assert_eq!(sup.components[pair].state, ProcState::Idle);
    }

    #[tokio::test]
    async fn starts_respect_selection_and_availability() {
        let mut sup = supervisor();
        let (base, pair, hardware) = (idx(&sup, "base"), idx(&sup, "pair"), idx(&sup, "hardware"));
        sup.set_enabled(base, false).unwrap();
        // A start of everything with a missing requirement is refused as a whole.
        let err = sup.start_all_enabled().unwrap_err();
        assert_eq!(
            err,
            ActionError::Conflict("pair requires base, which is disabled".into())
        );
        assert!(matches!(
            sup.start_component(pair),
            Err(ActionError::Conflict(_))
        ));

        sup.apply_profile("sim").unwrap();
        assert!(matches!(
            sup.start_component(hardware),
            Err(ActionError::Conflict(_))
        ));
    }

    #[tokio::test]
    async fn no_starts_after_shutdown_began() {
        let mut sup = supervisor();
        sup.begin_shutdown();
        assert!(matches!(
            sup.start_component(idx(&sup, "base")),
            Err(ActionError::Conflict(_))
        ));
    }

    #[test]
    fn locked_components_cannot_be_switched() {
        let mut sup = supervisor();
        let locked = idx(&sup, "locked");
        assert!(matches!(
            sup.set_enabled(locked, false),
            Err(ActionError::Conflict(_))
        ));
    }

    #[test]
    fn cli_applies_profile_then_parameters_then_switches() {
        let mut sup = supervisor();
        sup.apply_cli(
            Some("sim"),
            &["field=custom".into()],
            &["extra".into()],
            &[],
            &[],
        )
        .unwrap();
        let status = sup.status();
        assert_eq!(status.profile.as_deref(), Some("sim"));
        assert_eq!(status.parameters[0].value, Value::Bool(true));
        assert_eq!(status.parameters[1].value, Value::String("custom".into()));
        let extra = &status.components[idx(&sup, "extra")];
        assert!(extra.enabled && extra.active);
        assert!(!status.components[idx(&sup, "hardware")].available);
        assert_eq!(
            sup.stack.commands(idx(&sup, "extra"), &sup.values, ""),
            [["echo", "custom"]]
        );
    }

    #[test]
    fn cli_rejects_unknown_names_and_bad_values() {
        let mut sup = supervisor();
        assert!(sup.apply_cli(Some("nope"), &[], &[], &[], &[]).is_err());
        assert!(sup
            .apply_cli(None, &["sim=yes".into()], &[], &[], &[])
            .is_err());
        assert!(sup
            .apply_cli(None, &["nope=1".into()], &[], &[], &[])
            .is_err());
        assert!(sup.apply_cli(None, &["sim".into()], &[], &[], &[]).is_err());
        assert!(sup
            .apply_cli(None, &[], &["nope".into()], &[], &[])
            .is_err());
    }

    #[test]
    fn profiles_start_from_the_defaults() {
        let mut sup = supervisor();
        let extra = idx(&sup, "extra");
        sup.set_parameters(SetParameters::from([(
            "field".to_string(),
            Value::String("changed".into()),
        )]))
        .unwrap();
        sup.apply_profile("sim").unwrap();
        assert!(!sup.enabled[extra]);
        sup.set_enabled(extra, true).unwrap();
        sup.apply_profile("sim").unwrap();
        assert!(!sup.enabled[extra], "reapplying resets the selection");
        assert_eq!(sup.values["field"], Value::String("kid".into()));
    }

    #[test]
    fn global_parameters_reach_components_as_ros_parameter_file() {
        let mut sup = supervisor_for("globals");
        let board = idx(&sup, "board");
        let file = global_file("globals");
        // Without overrides the components get a valid file without parameters.
        let expected = [["board".to_string(), format!("file={}", file.display())]];
        assert_eq!(sup.component_commands(board).unwrap(), expected);
        let empty = serde_norway::Value::Mapping(Default::default());
        let params_in_file = || {
            let text = std::fs::read_to_string(&file).unwrap();
            let doc: serde_norway::Value = serde_norway::from_str(&text).unwrap();
            doc["/**"]["ros__parameters"].clone()
        };
        assert_eq!(params_in_file(), empty);

        sup.apply_cli(
            None,
            &[],
            &[],
            &[],
            &[
                "bot_id=2".into(),
                "field.size.x=6.5".into(),
                "role=goalie".into(),
            ],
        )
        .unwrap();
        assert_eq!(
            sup.stack
                .commands(board, &sup.values, &sup.global_parameters_path()),
            [["board".to_string(), format!("file={}", file.display())]]
        );
        let text = std::fs::read_to_string(&file).unwrap();
        let doc: serde_norway::Value = serde_norway::from_str(&text).unwrap();
        let params = &doc["/**"]["ros__parameters"];
        assert_eq!(params["bot_id"], serde_norway::Value::from(2));
        assert_eq!(params["field"]["size"]["x"], serde_norway::Value::from(6.5));
        assert_eq!(params["role"], serde_norway::Value::from("goalie"));

        // Removing the last override leaves a file without parameters.
        sup.set_global_parameters(SetGlobalParameters::from_iter(
            ["bot_id", "field.size.x", "role"].map(|n| (n.to_string(), None)),
        ))
        .unwrap();
        assert_eq!(params_in_file(), empty);
    }

    #[test]
    fn empty_launch_arguments_are_refused() {
        let mut sup = supervisor();
        let launched = idx(&sup, "launched");
        sup.set_parameters(SetParameters::from([(
            "field".to_string(),
            Value::String(String::new()),
        )]))
        .unwrap();
        let err = sup.start_component(launched).unwrap_err();
        assert!(
            matches!(&err, ActionError::Conflict(m) if m.contains("'field:=' would be empty")),
            "{err:?}"
        );
        assert_eq!(sup.components[launched].state, ProcState::Idle);
    }

    #[test]
    fn invalid_global_parameters_are_refused() {
        let mut sup = supervisor_for("globals_invalid");
        for name in ["", "a..b", "with space", ".x"] {
            let change =
                SetGlobalParameters::from([(name.to_string(), Some(GlobalValue::Integer(1)))]);
            assert!(
                matches!(
                    sup.set_global_parameters(change),
                    Err(ActionError::BadRequest(_))
                ),
                "{name:?}"
            );
        }
        // A parameter cannot also be the prefix of another one.
        let change = SetGlobalParameters::from([
            ("a".to_string(), Some(GlobalValue::Integer(1))),
            ("a.b".to_string(), Some(GlobalValue::Integer(2))),
        ]);
        assert!(matches!(
            sup.set_global_parameters(change),
            Err(ActionError::Failed(_))
        ));
        assert!(
            sup.global_parameters.is_empty(),
            "nothing is kept of a refused change"
        );
        assert!(sup
            .apply_cli(None, &[], &[], &[], &["no_value".into()])
            .is_err());
    }

    #[test]
    fn global_values_are_typed_by_yaml() {
        let parse = |raw| parse_global_value("p", raw).unwrap();
        assert_eq!(parse("2"), GlobalValue::Integer(2));
        assert_eq!(parse("2.0"), GlobalValue::Double(2.0));
        assert_eq!(parse("true"), GlobalValue::Bool(true));
        assert_eq!(parse("offense"), GlobalValue::String("offense".into()));
        assert_eq!(parse("'2'"), GlobalValue::String("2".into()));
        assert_eq!(parse("[1, 2]"), GlobalValue::IntegerArray(vec![1, 2]));
        assert!(parse_global_value("p", "{a: 1}").is_err());
    }

    #[test]
    fn parameter_updates_are_all_or_nothing() {
        let mut sup = supervisor();
        let update = SetParameters::from([
            ("field".to_string(), Value::String("other".into())),
            ("sim".to_string(), Value::String("not a bool".into())),
        ]);
        assert!(matches!(
            sup.set_parameters(update),
            Err(ActionError::BadRequest(_))
        ));
        assert_eq!(sup.values["field"], Value::String("lab".into()));
    }
}
