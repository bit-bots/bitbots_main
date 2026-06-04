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
            ProcState::Restarting => "↺   ",
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
    pub pids: Vec<u32>,
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
            task_handles: Vec::new(),
        }
    }
}

pub enum AppMsg {
    ProcessExited { key: String, code: i32 },
}

// ─── Screen / focus ───────────────────────────────────────────────────────────

#[derive(PartialEq, Clone)]
pub enum Screen {
    Config,
    Runtime,
    Logs(usize),
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
    pub all_logs: Arc<Mutex<VecDeque<String>>>,
    /// Interleaved stdout/stderr for headless mode; preserves ordering under load.
    pub headless_log: Arc<Mutex<VecDeque<HeadlessLine>>>,
    pub show_log_panel: bool,
}

impl App {
    pub fn new() -> Self {
        let (msg_tx, msg_rx) = mpsc::unbounded_channel();
        let components: Vec<ComponentState> = COMPONENT_DEFS
            .iter()
            .enumerate()
            .map(|(i, def)| {
                ComponentState::new(i, if def.sim_component { false } else { def.default_enabled })
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
            all_logs: Arc::new(Mutex::new(VecDeque::with_capacity(5000))),
            headless_log: Arc::new(Mutex::new(VecDeque::with_capacity(5000))),
            show_log_panel: true,
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
    ) {
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
            if let Some(idx) = COMPONENT_DEFS.iter().position(|d| d.key == "zenoh") {
                self.components[idx].enabled = false;
                self.params.zenoh = false;
            }
        }
        for key in enable {
            if let Some(idx) = COMPONENT_DEFS.iter().position(|d| d.key == key.as_str()) {
                self.components[idx].enabled = true;
            }
        }
        for key in disable {
            if let Some(idx) = COMPONENT_DEFS.iter().position(|d| d.key == key.as_str()) {
                self.components[idx].enabled = false;
            }
        }
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

    pub fn config_focus_items(&self) -> Vec<ConfigFocus> {
        let mut items = vec![
            ConfigFocus::Zenoh,
            ConfigFocus::Sim,
            ConfigFocus::Fieldname,
            ConfigFocus::DsdFile,
        ];
        for i in 0..self.toggleable_indices().len() {
            items.push(ConfigFocus::Component(i));
        }
        items.push(ConfigFocus::Start);
        items
    }

    pub fn config_focus_next(&mut self) {
        let items = self.config_focus_items();
        let pos = items.iter().position(|f| f == &self.config_focus).unwrap_or(0);
        self.config_focus = items[(pos + 1) % items.len()].clone();
    }

    pub fn config_focus_prev(&mut self) {
        let items = self.config_focus_items();
        let pos = items.iter().position(|f| f == &self.config_focus).unwrap_or(0);
        self.config_focus = items[(pos + items.len() - 1) % items.len()].clone();
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
        self.config_fieldname_input =
            if sim { FIELDNAME_DEFAULT_SIM } else { FIELDNAME_DEFAULT_HW }.to_string();
        self.params.fieldname = self.config_fieldname_input.clone();
        self.config_focus = ConfigFocus::Zenoh;
    }

    pub async fn start_component(&mut self, comp_idx: usize) {
        let def = &COMPONENT_DEFS[comp_idx];
        let cmds = (def.cmds)(&self.params);
        let key = def.key.to_string();
        let comp_name = def.name; // &'static str

        let logs = self.components[comp_idx].logs.clone();
        let all_logs = self.all_logs.clone();
        let headless_log = self.headless_log.clone();
        self.components[comp_idx].state = ProcState::Running;
        self.components[comp_idx].pids.clear();
        self.components[comp_idx].task_handles.clear();

        for cmd_args in cmds {
            if cmd_args.is_empty() {
                continue;
            }
            let tx = self.msg_tx.clone();
            let logs = logs.clone();
            let all_logs = all_logs.clone();
            let headless_log = headless_log.clone();
            let key = key.clone();

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
                    let entry = format!("[ERROR] spawn failed: {e}");
                    logs.lock().unwrap().push_back(entry.clone());
                    all_logs.lock().unwrap().push_back(format!("[{comp_name}] {entry}"));
                    let _ = tx.send(AppMsg::ProcessExited { key, code: -1 });
                    return;
                }
            };

            if let Some(pid) = child.id() {
                self.components[comp_idx].pids.push(pid);
            }

            let stdout = child.stdout.take().map(BufReader::new);
            let stderr = child.stderr.take().map(BufReader::new);

            let handle = tokio::spawn(async move {
                async fn pump<R: tokio::io::AsyncRead + Unpin>(
                    mut r: BufReader<R>,
                    logs: Arc<Mutex<VecDeque<String>>>,
                    all_logs: Arc<Mutex<VecDeque<String>>>,
                    headless_log: Arc<Mutex<VecDeque<HeadlessLine>>>,
                    is_stderr: bool,
                    comp_name: &'static str,
                ) {
                    let tui_prefix = if is_stderr { "[ERR] " } else { "" };
                    let mut line = String::new();
                    while let Ok(n) = r.read_line(&mut line).await {
                        if n == 0 {
                            break;
                        }
                        let raw = line.trim_end_matches('\n');
                        let tui_entry = format!("{}{}", tui_prefix, raw);
                        {
                            let mut l = logs.lock().unwrap();
                            if l.len() >= 2000 { l.pop_front(); }
                            l.push_back(tui_entry.clone());
                        }
                        {
                            let mut l = all_logs.lock().unwrap();
                            if l.len() >= 5000 { l.pop_front(); }
                            l.push_back(format!("[{comp_name}] {tui_entry}"));
                        }
                        {
                            let tagged = format!("[{comp_name}] {raw}");
                            let mut l = headless_log.lock().unwrap();
                            if l.len() >= 5000 { l.pop_front(); }
                            l.push_back(if is_stderr {
                                HeadlessLine::Stderr(tagged)
                            } else {
                                HeadlessLine::Stdout(tagged)
                            });
                        }
                        line.clear();
                    }
                }
                // Read stdout and stderr concurrently so neither blocks the other.
                let (logs2, all2, hl2) = (logs.clone(), all_logs.clone(), headless_log.clone());
                tokio::join!(
                    async move {
                        if let Some(r) = stdout {
                            pump(r, logs2, all2, hl2, false, comp_name).await;
                        }
                    },
                    async move {
                        if let Some(r) = stderr {
                            pump(r, logs, all_logs, headless_log, true, comp_name).await;
                        }
                    }
                );
                let code = child
                    .wait()
                    .await
                    .map(|s| s.code().unwrap_or(-1))
                    .unwrap_or(-1);
                let _ = tx.send(AppMsg::ProcessExited { key, code });
            });
            self.components[comp_idx].task_handles.push(handle);
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
    pub fn drain_messages(&mut self) -> Vec<usize> {
        let mut to_restart = Vec::new();
        while let Ok(msg) = self.msg_rx.try_recv() {
            match msg {
                AppMsg::ProcessExited { key, code } => {
                    if let Some((idx, comp)) = self
                        .components
                        .iter_mut()
                        .enumerate()
                        .find(|(_, c)| COMPONENT_DEFS[c.comp_idx].key == key)
                    {
                        match comp.state {
                            ProcState::Restarting => {
                                comp.pids.clear();
                                comp.state = ProcState::Idle;
                                to_restart.push(idx);
                            }
                            ProcState::Stopping | ProcState::Running => {
                                comp.state = if code == 0 {
                                    ProcState::Stopped
                                } else {
                                    ProcState::Crashed
                                };
                                comp.pids.clear();
                            }
                            _ => {}
                        }
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
