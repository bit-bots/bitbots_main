use std::{
    collections::VecDeque,
    sync::{Arc, Mutex},
};

use tokio::{
    io::{AsyncBufReadExt, BufReader},
    process::Command,
    sync::mpsc,
};

pub use crate::components::GlobalParams;
use crate::components::{COMPONENT_DEFS, FIELDNAME_DEFAULT_HW, FIELDNAME_DEFAULT_SIM};

// ─── State types ──────────────────────────────────────────────────────────────

pub enum HeadlessLine {
    Stdout(String),
    Stderr(String),
}

#[derive(Clone, Debug, PartialEq)]
pub enum ProcState {
    Idle,
    Running,
    /// SIGTERM sent, waiting for exit
    Stopping,
    /// SIGTERM sent, will restart automatically when the process exits
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
}

pub struct ComponentState {
    pub comp_idx: usize,
    pub enabled: bool,
    pub state: ProcState,
    pub logs: Arc<Mutex<VecDeque<String>>>,
    /// Process group ids of the processes of the current run that have not exited yet.
    pub pids: Vec<u32>,
    /// First exit code reported in the current run; decides between Stopped and
    /// Crashed once all processes of the component have exited.
    pub exit_code: Option<i32>,
    pub task_handles: Vec<tokio::task::JoinHandle<()>>,
}

impl ComponentState {
    pub fn new(comp_idx: usize, enabled: bool) -> Self {
        Self {
            comp_idx,
            enabled,
            state: ProcState::Idle,
            logs: Arc::new(Mutex::new(VecDeque::with_capacity(2000))),
            pids: Vec::new(),
            exit_code: None,
            task_handles: Vec::new(),
        }
    }
}

pub enum AppMsg {
    ProcessExited {
        comp_idx: usize,
        pid: u32,
        code: i32,
    },
}

/// The log buffers a component's output is written to.
#[derive(Clone)]
struct LogSinks {
    comp_name: &'static str,
    logs: Arc<Mutex<VecDeque<String>>>,
    all_logs: Arc<Mutex<VecDeque<String>>>,
    headless_log: Arc<Mutex<VecDeque<HeadlessLine>>>,
}

impl LogSinks {
    fn push(&self, raw: &str, is_stderr: bool) {
        let comp_name = self.comp_name;
        let tui_prefix = if is_stderr { "[ERR] " } else { "" };
        let tui_entry = format!("{tui_prefix}{raw}");
        {
            let mut l = self.logs.lock().unwrap();
            if l.len() >= 2000 {
                l.pop_front();
            }
            l.push_back(tui_entry.clone());
        }
        {
            let mut l = self.all_logs.lock().unwrap();
            if l.len() >= 5000 {
                l.pop_front();
            }
            l.push_back(format!("[{comp_name}] {tui_entry}"));
        }
        {
            let tagged = format!("[{comp_name}] {raw}");
            let mut l = self.headless_log.lock().unwrap();
            if l.len() >= 5000 {
                l.pop_front();
            }
            l.push_back(if is_stderr {
                HeadlessLine::Stderr(tagged)
            } else {
                HeadlessLine::Stdout(tagged)
            });
        }
    }

    /// Messages from the launcher itself, as opposed to output of the child processes.
    fn push_teamplayer(&self, msg: &str) {
        self.push(&format!("[teamplayer] {msg}"), true);
    }

    async fn pump<R: tokio::io::AsyncRead + Unpin>(&self, mut r: BufReader<R>, is_stderr: bool) {
        let mut line = String::new();
        while let Ok(n) = r.read_line(&mut line).await {
            if n == 0 {
                break;
            }
            self.push(line.trim_end_matches('\n'), is_stderr);
            line.clear();
        }
    }
}

pub fn component_idx(key: &str) -> Option<usize> {
    COMPONENT_DEFS.iter().position(|d| d.key == key)
}

// ─── Screen / focus ───────────────────────────────────────────────────────────

#[derive(PartialEq, Clone)]
pub enum Screen {
    Config,
    Runtime,
    Logs(usize),
    AllLogs,
}

pub enum ConfirmAction {
    Quit,
    StopAll,
}

#[derive(Clone, PartialEq, Debug)]
pub enum ConfigFocus {
    Zenoh,
    Sim,
    Fieldname,
    DsdFile,
    Component(usize),
    Start,
}

// ─── App ──────────────────────────────────────────────────────────────────────

pub struct App {
    pub screen: Screen,
    pub params: GlobalParams,
    pub components: Vec<ComponentState>,
    pub config_focus: ConfigFocus,
    pub config_fieldname_input: String,
    pub config_dsd_input: String,
    pub runtime_selected: usize,
    pub tick_count: u64,
    pub msg_tx: mpsc::UnboundedSender<AppMsg>,
    pub msg_rx: mpsc::UnboundedReceiver<AppMsg>,
    pub log_scroll: usize,
    pub log_follow: bool,
    /// Viewport height written by the draw function so scroll_keys can clamp correctly.
    pub log_viewport_height: std::cell::Cell<usize>,
    pub all_logs: Arc<Mutex<VecDeque<String>>>,
    /// Interleaved stdout/stderr for headless mode; preserves ordering under load.
    pub headless_log: Arc<Mutex<VecDeque<HeadlessLine>>>,
    pub show_log_panel: bool,
    pub confirm_action: Option<ConfirmAction>,
}

impl App {
    pub fn new() -> Self {
        let (msg_tx, msg_rx) = mpsc::unbounded_channel();
        let components: Vec<ComponentState> = COMPONENT_DEFS
            .iter()
            .enumerate()
            .map(|(i, def)| {
                ComponentState::new(
                    i,
                    if def.sim_component {
                        false
                    } else {
                        def.default_enabled
                    },
                )
            })
            .collect();

        App {
            screen: Screen::Config,
            params: GlobalParams::default(),
            components,
            config_focus: ConfigFocus::Zenoh,
            config_fieldname_input: FIELDNAME_DEFAULT_HW.to_string(),
            config_dsd_input: "main.dsd".to_string(),
            runtime_selected: 0,
            tick_count: 0,
            msg_tx,
            msg_rx,
            log_scroll: 0,
            log_follow: true,
            log_viewport_height: std::cell::Cell::new(0),
            all_logs: Arc::new(Mutex::new(VecDeque::with_capacity(5000))),
            headless_log: Arc::new(Mutex::new(VecDeque::with_capacity(5000))),
            show_log_panel: true,
            confirm_action: None,
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
        if sim {
            self.params.sim = true;
            self.apply_sim_toggle();
        }
        if let Some(f) = fieldname {
            self.params.fieldname = f.to_string();
            self.config_fieldname_input = f.to_string();
        }
        self.params.dsd_file = dsd_file.to_string();
        self.config_dsd_input = dsd_file.to_string();

        if no_zenoh {
            self.set_zenoh_enabled(false);
        }
        for (keys, enabled) in [(enable, true), (disable, false)] {
            for key in keys {
                let Some(idx) = component_idx(key) else {
                    let available: Vec<&str> = COMPONENT_DEFS.iter().map(|d| d.key).collect();
                    anyhow::bail!(
                        "unknown component key '{key}', available keys: {}",
                        available.join(", ")
                    );
                };
                self.components[idx].enabled = enabled;
            }
        }
        Ok(())
    }

    fn zenoh_idx() -> usize {
        component_idx("zenoh").expect("the component registry contains zenoh")
    }

    pub fn zenoh_enabled(&self) -> bool {
        self.components[Self::zenoh_idx()].enabled
    }

    pub fn set_zenoh_enabled(&mut self, enabled: bool) {
        self.components[Self::zenoh_idx()].enabled = enabled;
    }

    pub fn toggleable_indices(&self) -> Vec<usize> {
        COMPONENT_DEFS
            .iter()
            .enumerate()
            .filter(|(_, d)| {
                !d.infrastructure && !d.sim_component && !(d.hardware_only && self.params.sim)
            })
            .map(|(i, _)| i)
            .collect()
    }

    /// All focusable config items in reading order: flags row, left column, right column, Start.
    fn config_focus_order(&self) -> Vec<ConfigFocus> {
        let n = self.toggleable_indices().len();
        [
            ConfigFocus::Zenoh,
            ConfigFocus::Sim,
            ConfigFocus::Fieldname,
            ConfigFocus::DsdFile,
        ]
        .into_iter()
        .chain((0..n).map(ConfigFocus::Component))
        .chain([ConfigFocus::Start])
        .collect()
    }

    /// Tab: cycle forward through every item in reading order.
    pub fn config_focus_tab(&mut self) {
        let order = self.config_focus_order();
        let pos = order
            .iter()
            .position(|f| *f == self.config_focus)
            .unwrap_or(0);
        self.config_focus = order[(pos + 1) % order.len()].clone();
    }

    /// Shift-Tab: cycle backward through every item in reading order.
    pub fn config_focus_backtab(&mut self) {
        let order = self.config_focus_order();
        let pos = order
            .iter()
            .position(|f| *f == self.config_focus)
            .unwrap_or(0);
        self.config_focus = order[(pos + order.len() - 1) % order.len()].clone();
    }

    /// Down: flags row → first component; then down the left column, continuing at the top
    /// of the right column; bottom of the right column → Start; Start wraps to flags.
    pub fn config_focus_next(&mut self) {
        let n = self.toggleable_indices().len();
        self.config_focus = match self.config_focus.clone() {
            ConfigFocus::Zenoh
            | ConfigFocus::Sim
            | ConfigFocus::Fieldname
            | ConfigFocus::DsdFile => {
                if n > 0 {
                    ConfigFocus::Component(0)
                } else {
                    ConfigFocus::Start
                }
            }
            ConfigFocus::Component(i) => {
                if i + 1 < n {
                    ConfigFocus::Component(i + 1)
                } else {
                    ConfigFocus::Start
                }
            }
            ConfigFocus::Start => ConfigFocus::Zenoh,
        };
    }

    /// Up: exact inverse of Down, except that the flags row wraps to Start.
    pub fn config_focus_prev(&mut self) {
        let n = self.toggleable_indices().len();
        self.config_focus = match self.config_focus.clone() {
            ConfigFocus::Zenoh
            | ConfigFocus::Sim
            | ConfigFocus::Fieldname
            | ConfigFocus::DsdFile => ConfigFocus::Start,
            ConfigFocus::Component(0) => ConfigFocus::Zenoh,
            ConfigFocus::Component(i) => ConfigFocus::Component(i - 1),
            ConfigFocus::Start => {
                if n > 0 {
                    ConfigFocus::Component(n - 1)
                } else {
                    ConfigFocus::Zenoh
                }
            }
        };
    }

    pub fn config_focus_right(&mut self) {
        let n = self.toggleable_indices().len();
        let left_count = (n + 1) / 2;
        match self.config_focus.clone() {
            ConfigFocus::Zenoh => self.config_focus = ConfigFocus::Sim,
            ConfigFocus::Sim => self.config_focus = ConfigFocus::Fieldname,
            ConfigFocus::Fieldname => self.config_focus = ConfigFocus::DsdFile,
            ConfigFocus::Component(i) if i < left_count => {
                let right = i + left_count;
                if right < n {
                    self.config_focus = ConfigFocus::Component(right);
                }
            }
            _ => {}
        }
    }

    pub fn config_focus_left(&mut self) {
        let left_count = (self.toggleable_indices().len() + 1) / 2;
        match self.config_focus.clone() {
            ConfigFocus::Sim => self.config_focus = ConfigFocus::Zenoh,
            ConfigFocus::Fieldname => self.config_focus = ConfigFocus::Sim,
            ConfigFocus::DsdFile => self.config_focus = ConfigFocus::Fieldname,
            ConfigFocus::Component(i) if i >= left_count => {
                self.config_focus = ConfigFocus::Component(i - left_count);
            }
            _ => {}
        }
    }

    pub fn runtime_visible_indices(&self) -> Vec<usize> {
        COMPONENT_DEFS
            .iter()
            .enumerate()
            .filter(|(i, d)| {
                if d.sim_component {
                    return self.params.sim;
                }
                if d.hardware_only && self.params.sim {
                    return false;
                }
                self.components[*i].enabled
            })
            .map(|(i, _)| i)
            .collect()
    }

    pub fn apply_sim_toggle(&mut self) {
        let sim = self.params.sim;
        for (i, def) in COMPONENT_DEFS.iter().enumerate() {
            if def.sim_component {
                self.components[i].enabled = sim;
            } else if def.hardware_only {
                self.components[i].enabled = !sim && def.default_enabled;
            }
        }
        self.config_fieldname_input = if sim {
            FIELDNAME_DEFAULT_SIM
        } else {
            FIELDNAME_DEFAULT_HW
        }
        .to_string();
        self.params.fieldname = self.config_fieldname_input.clone();
        self.config_focus = ConfigFocus::Zenoh;
    }

    fn log_sinks(&self, comp_idx: usize) -> LogSinks {
        LogSinks {
            comp_name: COMPONENT_DEFS[comp_idx].name,
            logs: self.components[comp_idx].logs.clone(),
            all_logs: self.all_logs.clone(),
            headless_log: self.headless_log.clone(),
        }
    }

    pub async fn start_component(&mut self, comp_idx: usize) {
        let cmds = (COMPONENT_DEFS[comp_idx].cmds)(&self.params);
        self.spawn_commands(comp_idx, cmds);
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
        let sinks = self.log_sinks(comp_idx);
        let msg_tx = self.msg_tx.clone();
        let comp = &mut self.components[comp_idx];
        comp.state = ProcState::Running;
        comp.exit_code = None;
        comp.task_handles.clear();

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
                    sinks
                        .push_teamplayer(&format!("failed to spawn `{}`: {e}", cmd_args.join(" ")));
                    comp.exit_code = Some(-1);
                    if comp.pids.is_empty() {
                        comp.state = ProcState::Crashed;
                    } else {
                        sinks.push_teamplayer("stopping the already started processes");
                        sigterm_pids(&comp.pids);
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
            let sinks = sinks.clone();

            let handle = tokio::spawn(async move {
                // Read stdout and stderr concurrently so neither blocks the other.
                tokio::join!(
                    async {
                        if let Some(r) = stdout {
                            sinks.pump(r, false).await;
                        }
                    },
                    async {
                        if let Some(r) = stderr {
                            sinks.pump(r, true).await;
                        }
                    }
                );
                let code = child
                    .wait()
                    .await
                    .map(|s| s.code().unwrap_or(-1))
                    .unwrap_or(-1);
                let _ = tx.send(AppMsg::ProcessExited {
                    comp_idx,
                    pid,
                    code,
                });
            });
            comp.task_handles.push(handle);
        }
    }

    /// Send SIGTERM to the component's direct child processes and mark as Stopping.
    pub async fn stop_component(&mut self, comp_idx: usize) {
        match self.components[comp_idx].state {
            ProcState::Running => {
                sigterm_pids(&self.components[comp_idx].pids);
                self.components[comp_idx].state = ProcState::Stopping;
            }
            ProcState::Restarting => {
                // Already sent SIGTERM — cancel the pending restart
                self.components[comp_idx].state = ProcState::Stopping;
            }
            _ => {}
        }
    }

    /// Send SIGTERM and mark as Restarting — will auto-restart when the process exits.
    pub fn restart_component(&mut self, comp_idx: usize) {
        if self.components[comp_idx].state != ProcState::Running {
            return;
        }
        sigterm_pids(&self.components[comp_idx].pids);
        self.components[comp_idx].state = ProcState::Restarting;
    }

    pub async fn stop_all(&mut self) {
        for i in 0..self.components.len() {
            self.stop_component(i).await;
        }
    }

    pub async fn start_all_enabled(&mut self) {
        let enabled: Vec<usize> = self
            .components
            .iter()
            .enumerate()
            .filter(|(_, c)| c.enabled && c.state == ProcState::Idle)
            .map(|(i, _)| i)
            .collect();
        for i in enabled {
            self.start_component(i).await;
        }
    }

    /// Drain the message queue and return the indices of components that need restarting.
    ///
    /// A component only leaves Running/Stopping/Restarting once all of its processes have
    /// exited, so a multi-process component is never restarted while parts of it still run.
    pub fn drain_messages(&mut self) -> Vec<usize> {
        let mut to_restart = Vec::new();
        while let Ok(msg) = self.msg_rx.try_recv() {
            match msg {
                AppMsg::ProcessExited {
                    comp_idx,
                    pid,
                    code,
                } => {
                    let sinks = self.log_sinks(comp_idx);
                    let comp = &mut self.components[comp_idx];
                    let Some(pos) = comp.pids.iter().position(|&p| p == pid) else {
                        sinks.push_teamplayer(&format!(
                            "exit of untracked pid {pid} (code {code}) reported, ignoring it"
                        ));
                        continue;
                    };
                    comp.pids.swap_remove(pos);
                    comp.exit_code.get_or_insert(code);

                    if comp.state == ProcState::Running {
                        sinks.push_teamplayer(&format!(
                            "process {pid} exited unexpectedly with code {code}"
                        ));
                        if !comp.pids.is_empty() {
                            // Do not leave the component half running.
                            sinks.push_teamplayer("stopping the remaining processes");
                            sigterm_pids(&comp.pids);
                            comp.state = ProcState::Stopping;
                        }
                    }

                    if !comp.pids.is_empty() {
                        continue;
                    }
                    match comp.state {
                        ProcState::Restarting => {
                            comp.state = ProcState::Idle;
                            to_restart.push(comp_idx);
                        }
                        ProcState::Stopping | ProcState::Running => {
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

fn sigterm_pids(pids: &[u32]) {
    use nix::sys::signal::{killpg, Signal};
    use nix::unistd::Pid;
    for &pid in pids {
        let _ = killpg(Pid::from_raw(pid as i32), Signal::SIGTERM);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn cmd(args: &[&str]) -> Vec<String> {
        args.iter().map(|s| s.to_string()).collect()
    }

    fn world_model_idx() -> usize {
        component_idx("world_model").unwrap()
    }

    /// Drain messages until the component settles in a non-transitional state.
    async fn wait_settled(app: &mut App, comp_idx: usize) -> Vec<usize> {
        let mut to_restart = Vec::new();
        for _ in 0..200 {
            to_restart.extend(app.drain_messages());
            if !to_restart.is_empty()
                || !matches!(
                    app.components[comp_idx].state,
                    ProcState::Running | ProcState::Stopping | ProcState::Restarting
                )
            {
                return to_restart;
            }
            tokio::time::sleep(Duration::from_millis(25)).await;
        }
        panic!(
            "component did not settle, state {:?}",
            app.components[comp_idx].state
        );
    }

    #[tokio::test]
    async fn restart_waits_for_all_processes_of_a_component() {
        let mut app = App::new();
        let idx = world_model_idx();
        app.spawn_commands(idx, vec![cmd(&["sleep", "30"]), cmd(&["sleep", "30"])]);
        assert_eq!(app.components[idx].pids.len(), 2);

        app.restart_component(idx);
        let to_restart = wait_settled(&mut app, idx).await;
        assert_eq!(to_restart, vec![idx]);
        assert!(app.components[idx].pids.is_empty());

        // The new run must not be affected by exit reports of the old one.
        app.spawn_commands(idx, vec![cmd(&["sleep", "30"]), cmd(&["sleep", "30"])]);
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert!(app.drain_messages().is_empty());
        assert_eq!(app.components[idx].state, ProcState::Running);
        assert_eq!(app.components[idx].pids.len(), 2);

        app.stop_component(idx).await;
        wait_settled(&mut app, idx).await;
        assert!(app.components[idx].pids.is_empty());
    }

    #[tokio::test]
    async fn partial_spawn_failure_stops_started_processes() {
        let mut app = App::new();
        let idx = world_model_idx();
        app.spawn_commands(
            idx,
            vec![
                cmd(&["sleep", "30"]),
                cmd(&["/nonexistent/teamplayer-test-binary"]),
            ],
        );
        assert_eq!(app.components[idx].state, ProcState::Stopping);

        wait_settled(&mut app, idx).await;
        assert_eq!(app.components[idx].state, ProcState::Crashed);
        assert!(app.components[idx].pids.is_empty());
    }

    #[tokio::test]
    async fn spawn_failure_of_only_process_crashes_immediately() {
        let mut app = App::new();
        let idx = world_model_idx();
        app.spawn_commands(idx, vec![cmd(&["/nonexistent/teamplayer-test-binary"])]);
        assert_eq!(app.components[idx].state, ProcState::Crashed);
        assert!(app.components[idx].pids.is_empty());
    }

    #[tokio::test]
    async fn crash_of_one_process_stops_its_siblings() {
        let mut app = App::new();
        let idx = world_model_idx();
        app.spawn_commands(
            idx,
            vec![cmd(&["sh", "-c", "exit 3"]), cmd(&["sleep", "30"])],
        );
        wait_settled(&mut app, idx).await;
        assert_eq!(app.components[idx].state, ProcState::Crashed);
        assert_eq!(app.components[idx].exit_code, Some(3));
        assert!(app.components[idx].pids.is_empty());
    }

    #[tokio::test]
    async fn clean_exit_is_reported_as_stopped() {
        let mut app = App::new();
        let idx = world_model_idx();
        app.spawn_commands(idx, vec![cmd(&["true"])]);
        wait_settled(&mut app, idx).await;
        assert_eq!(app.components[idx].state, ProcState::Stopped);
    }

    #[test]
    fn cli_zenoh_flags_share_one_state() {
        let mut app = App::new();
        app.apply_cli_presets(false, true, None, "main.dsd", &["zenoh".into()], &[])
            .unwrap();
        assert!(app.zenoh_enabled());
        app.apply_cli_presets(false, false, None, "main.dsd", &[], &["zenoh".into()])
            .unwrap();
        assert!(!app.zenoh_enabled());
    }

    #[test]
    fn cli_rejects_unknown_component_keys() {
        let mut app = App::new();
        assert!(app
            .apply_cli_presets(false, false, None, "main.dsd", &["nope".into()], &[])
            .is_err());
        assert!(app
            .apply_cli_presets(false, false, None, "main.dsd", &[], &["nope".into()])
            .is_err());
    }

    #[test]
    fn down_and_up_visit_every_component_and_are_inverse() {
        let mut app = App::new();
        let n = app.toggleable_indices().len();
        assert!(n >= 2, "test needs both config columns populated");

        app.config_focus = ConfigFocus::Zenoh;
        let mut visited = Vec::new();
        loop {
            app.config_focus_next();
            if app.config_focus == ConfigFocus::Start {
                break;
            }
            visited.push(app.config_focus.clone());
        }
        let expected: Vec<ConfigFocus> = (0..n).map(ConfigFocus::Component).collect();
        assert_eq!(visited, expected);

        for i in (0..n).rev() {
            app.config_focus_prev();
            assert_eq!(app.config_focus, ConfigFocus::Component(i));
        }
        app.config_focus_prev();
        assert_eq!(app.config_focus, ConfigFocus::Zenoh);
    }

    #[test]
    fn tab_cycles_through_all_items() {
        let mut app = App::new();
        let order = app.config_focus_order();
        app.config_focus = order[0].clone();
        for expected in order.iter().skip(1).chain(order.first()) {
            app.config_focus_tab();
            assert_eq!(&app.config_focus, expected);
        }
        app.config_focus_backtab();
        assert_eq!(&app.config_focus, order.last().unwrap());
    }

    #[test]
    fn left_right_move_between_columns() {
        let mut app = App::new();
        let n = app.toggleable_indices().len();
        let left_count = n.div_ceil(2);
        app.config_focus = ConfigFocus::Component(0);
        app.config_focus_right();
        assert_eq!(app.config_focus, ConfigFocus::Component(left_count));
        app.config_focus_left();
        assert_eq!(app.config_focus, ConfigFocus::Component(0));
    }
}
