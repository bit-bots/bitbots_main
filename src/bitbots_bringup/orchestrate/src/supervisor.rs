//! Starts, stops and restarts the components and keeps track of their processes.

use std::sync::Arc;

use anyhow::bail;
use tokio::{
    io::{AsyncBufReadExt, BufReader},
    process::Command,
    sync::mpsc,
};

use crate::{
    api::{ComponentInfo, Config, ConfigUpdate, LogStream, ProcState, Status},
    components::{component_idx, COMPONENT_DEFS, FIELDNAME_DEFAULT_HW, FIELDNAME_DEFAULT_SIM},
    logs::LogStore,
};

pub struct ComponentState {
    pub enabled: bool,
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
}

impl std::fmt::Display for ActionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ActionError::Conflict(msg) | ActionError::BadRequest(msg) => f.write_str(msg),
        }
    }
}

/// Writes the output of one component to the [`LogStore`].
#[derive(Clone)]
struct ComponentLog {
    key: &'static str,
    name: &'static str,
    logs: Arc<LogStore>,
}

impl ComponentLog {
    fn push(&self, stream: LogStream, text: &str) {
        self.logs.push(self.key, self.name, stream, text);
    }

    /// Messages from the orchestrator itself, as opposed to output of the child processes.
    fn push_orchestrate(&self, msg: &str) {
        self.push(LogStream::Stderr, &format!("[orchestrate] {msg}"));
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
    pub config: Config,
    pub components: Vec<ComponentState>,
    /// Set once shutdown has begun; no components are started afterwards.
    shutting_down: bool,
    logs: Arc<LogStore>,
    msg_tx: mpsc::UnboundedSender<SupervisorMsg>,
    msg_rx: mpsc::UnboundedReceiver<SupervisorMsg>,
}

impl Supervisor {
    pub fn new(logs: Arc<LogStore>) -> Self {
        let (msg_tx, msg_rx) = mpsc::unbounded_channel();
        let components = COMPONENT_DEFS
            .iter()
            .map(|def| ComponentState {
                enabled: !def.sim_component && def.default_enabled,
                state: ProcState::Idle,
                pids: Vec::new(),
                exit_code: None,
            })
            .collect();
        Self {
            config: Config::default(),
            components,
            shutting_down: false,
            logs,
            msg_tx,
            msg_rx,
        }
    }

    pub fn apply_cli_presets(
        &mut self,
        sim: bool,
        no_zenoh: bool,
        fieldname: Option<&str>,
        dsd_file: &str,
        enable: &[String],
        disable: &[String],
    ) -> anyhow::Result<()> {
        let update = ConfigUpdate {
            sim: Some(sim),
            fieldname: fieldname.map(str::to_string),
            dsd_file: Some(dsd_file.to_string()),
        };
        if let Err(e) = self.update_config(update) {
            bail!("{e}");
        }
        if no_zenoh {
            self.set_enabled(Self::zenoh_idx(), false);
        }
        for (keys, enabled) in [(enable, true), (disable, false)] {
            for key in keys {
                let Some(idx) = component_idx(key) else {
                    let available: Vec<&str> = COMPONENT_DEFS.iter().map(|d| d.key).collect();
                    bail!(
                        "unknown component key '{key}', available keys: {}",
                        available.join(", ")
                    );
                };
                self.set_enabled(idx, enabled);
            }
        }
        Ok(())
    }

    fn zenoh_idx() -> usize {
        component_idx("zenoh").expect("the component registry contains zenoh")
    }

    /// Takes effect on the next start of the affected components.
    pub fn update_config(&mut self, update: ConfigUpdate) -> Result<(), ActionError> {
        for (field, value) in [
            ("fieldname", &update.fieldname),
            ("dsd_file", &update.dsd_file),
        ] {
            if value.as_deref().is_some_and(|v| v.trim().is_empty()) {
                return Err(ActionError::BadRequest(format!(
                    "{field} must not be empty"
                )));
            }
        }
        if let Some(sim) = update.sim {
            if sim != self.config.sim {
                self.config.sim = sim;
                for (comp, def) in self.components.iter_mut().zip(COMPONENT_DEFS) {
                    if def.sim_component {
                        comp.enabled = sim;
                    } else if def.hardware_only {
                        comp.enabled = !sim && def.default_enabled;
                    }
                }
                self.config.fieldname = if sim {
                    FIELDNAME_DEFAULT_SIM
                } else {
                    FIELDNAME_DEFAULT_HW
                }
                .to_string();
            }
        }
        if let Some(fieldname) = update.fieldname {
            self.config.fieldname = fieldname;
        }
        if let Some(dsd_file) = update.dsd_file {
            self.config.dsd_file = dsd_file;
        }
        Ok(())
    }

    pub fn set_enabled(&mut self, comp_idx: usize, enabled: bool) {
        self.components[comp_idx].enabled = enabled;
    }

    pub fn status(&self) -> Status {
        Status {
            config: self.config.clone(),
            components: self
                .components
                .iter()
                .zip(COMPONENT_DEFS)
                .map(|(comp, def)| ComponentInfo {
                    key: def.key.to_string(),
                    name: def.name.to_string(),
                    enabled: comp.enabled,
                    state: comp.state,
                    pids: comp.pids.clone(),
                    exit_code: comp.exit_code,
                    infrastructure: def.infrastructure,
                    hardware_only: def.hardware_only,
                    sim_component: def.sim_component,
                })
                .collect(),
        }
    }

    pub fn any_alive(&self) -> bool {
        self.components.iter().any(|c| c.state.is_alive())
    }

    fn component_log(&self, comp_idx: usize) -> ComponentLog {
        ComponentLog {
            key: COMPONENT_DEFS[comp_idx].key,
            name: COMPONENT_DEFS[comp_idx].name,
            logs: self.logs.clone(),
        }
    }

    pub fn start_component(&mut self, comp_idx: usize) -> Result<(), ActionError> {
        let def = &COMPONENT_DEFS[comp_idx];
        if self.shutting_down {
            return Err(ActionError::Conflict(
                "orchestrate is shutting down".to_string(),
            ));
        }
        if self.components[comp_idx].state.is_alive() {
            return Err(ActionError::Conflict(format!(
                "{} is already running",
                def.key
            )));
        }
        let cmds = (def.cmds)(&self.config);
        self.spawn_commands(comp_idx, cmds);
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
            COMPONENT_DEFS[comp_idx].key
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
                    log.push_orchestrate(&format!("failed to spawn `{}`: {e}", cmd_args.join(" ")));
                    comp.exit_code = Some(-1);
                    if comp.pids.is_empty() {
                        comp.state = ProcState::Crashed;
                    } else {
                        log.push_orchestrate("stopping the already started processes");
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
                COMPONENT_DEFS[comp_idx].key
            ))),
            _ => self.start_component(comp_idx),
        }
    }

    pub fn stop_all(&mut self) {
        for i in 0..self.components.len() {
            self.stop_component(i);
        }
    }

    /// Start all enabled components that are not running.
    pub fn start_all_enabled(&mut self) -> Result<(), ActionError> {
        let to_start: Vec<usize> = self
            .components
            .iter()
            .enumerate()
            .filter(|(_, c)| c.enabled && !c.state.is_alive())
            .map(|(i, _)| i)
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
        for (comp, def) in self.components.iter().zip(COMPONENT_DEFS) {
            if !comp.pids.is_empty() {
                self.logs.push(
                    def.key,
                    def.name,
                    LogStream::Stderr,
                    &format!(
                        "[orchestrate] processes {:?} did not exit after SIGTERM, sending SIGKILL",
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
                    .push_orchestrate(&format!("restart failed: {e}"));
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
                        log.push_orchestrate(&format!(
                            "exit of untracked pid {pid} (code {code}) reported, ignoring it"
                        ));
                        continue;
                    };
                    comp.pids.swap_remove(pos);
                    comp.exit_code.get_or_insert(code);

                    if comp.state == ProcState::Running {
                        log.push_orchestrate(&format!(
                            "process {pid} exited unexpectedly with code {code}"
                        ));
                        if !comp.pids.is_empty() {
                            // Do not leave the component half running.
                            log.push_orchestrate("stopping the remaining processes");
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
    use std::time::Duration;

    fn cmd(args: &[&str]) -> Vec<String> {
        args.iter().map(|s| s.to_string()).collect()
    }

    fn supervisor() -> Supervisor {
        Supervisor::new(Arc::new(LogStore::new()))
    }

    fn world_model_idx() -> usize {
        component_idx("world_model").unwrap()
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
        let idx = world_model_idx();
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
        let idx = world_model_idx();
        sup.spawn_commands(
            idx,
            vec![
                cmd(&["sleep", "30"]),
                cmd(&["/nonexistent/orchestrate-test-binary"]),
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
        let idx = world_model_idx();
        sup.spawn_commands(idx, vec![cmd(&["/nonexistent/orchestrate-test-binary"])]);
        assert_eq!(sup.components[idx].state, ProcState::Crashed);
        assert!(sup.components[idx].pids.is_empty());
    }

    #[tokio::test]
    async fn crash_of_one_process_stops_its_siblings() {
        let mut sup = supervisor();
        let idx = world_model_idx();
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
        let idx = world_model_idx();
        sup.spawn_commands(idx, vec![cmd(&["true"])]);
        wait_settled(&mut sup, idx).await;
        assert_eq!(sup.components[idx].state, ProcState::Stopped);
    }

    #[tokio::test]
    async fn output_and_spawn_errors_reach_the_log_store() {
        let logs = Arc::new(LogStore::new());
        let mut sup = Supervisor::new(logs.clone());
        let idx = world_model_idx();
        sup.spawn_commands(idx, vec![cmd(&["sh", "-c", "echo out; echo err >&2"])]);
        wait_settled(&mut sup, idx).await;
        sup.spawn_commands(idx, vec![cmd(&["/nonexistent/orchestrate-test-binary"])]);

        let lines = logs.since(0, Some("world_model"), usize::MAX).lines;
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
    async fn no_starts_after_shutdown_began() {
        let mut sup = supervisor();
        sup.begin_shutdown();
        assert!(matches!(
            sup.start_component(world_model_idx()),
            Err(ActionError::Conflict(_))
        ));
    }

    #[test]
    fn cli_zenoh_flags_share_one_state() {
        let mut sup = supervisor();
        let zenoh = Supervisor::zenoh_idx();
        sup.apply_cli_presets(false, true, None, "main.dsd", &["zenoh".into()], &[])
            .unwrap();
        assert!(sup.components[zenoh].enabled);
        sup.apply_cli_presets(false, false, None, "main.dsd", &[], &["zenoh".into()])
            .unwrap();
        assert!(!sup.components[zenoh].enabled);
    }

    #[test]
    fn cli_rejects_unknown_component_keys() {
        let mut sup = supervisor();
        assert!(sup
            .apply_cli_presets(false, false, None, "main.dsd", &["nope".into()], &[])
            .is_err());
        assert!(sup
            .apply_cli_presets(false, false, None, "main.dsd", &[], &["nope".into()])
            .is_err());
    }

    #[test]
    fn sim_switch_toggles_components_and_fieldname() {
        let mut sup = supervisor();
        let simulator = component_idx("simulator").unwrap();
        let lowlevel = component_idx("lowlevel").unwrap();
        sup.update_config(ConfigUpdate {
            sim: Some(true),
            ..Default::default()
        })
        .unwrap();
        assert!(sup.components[simulator].enabled);
        assert!(!sup.components[lowlevel].enabled);
        assert_eq!(sup.config.fieldname, FIELDNAME_DEFAULT_SIM);

        // An explicit fieldname in the same update wins over the mode default.
        sup.update_config(ConfigUpdate {
            sim: Some(false),
            fieldname: Some("custom".into()),
            ..Default::default()
        })
        .unwrap();
        assert!(!sup.components[simulator].enabled);
        assert_eq!(sup.config.fieldname, "custom");
    }

    #[test]
    fn empty_config_values_are_rejected() {
        let mut sup = supervisor();
        let err = sup.update_config(ConfigUpdate {
            fieldname: Some(" ".into()),
            ..Default::default()
        });
        assert!(matches!(err, Err(ActionError::BadRequest(_))));
        assert_eq!(sup.config.fieldname, FIELDNAME_DEFAULT_HW);
    }
}
