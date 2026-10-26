use std::collections::VecDeque;

use bitbots_orchestrate::{
    api::{ComponentInfo, LogStream, Logs, Status},
    client::Client,
};

use crate::spawn::OwnedOrchestrator;

const ALL_LOGS_CAPACITY: usize = 5000;
const COMPONENT_LOGS_CAPACITY: usize = 5000;

// ─── Screen / focus ───────────────────────────────────────────────────────────

#[derive(PartialEq, Clone)]
pub enum Screen {
    Config,
    Runtime,
    Logs(usize),
    AllLogs,
}

pub enum ConfirmAction {
    StopAll,
    /// Quit while owning the orchestrator: stop it or leave it running
    Quit,
}

#[derive(Debug, PartialEq)]
pub enum ExitMode {
    /// Leave the orchestrator and its components running
    Detach,
    /// Stop the orchestrator this TUI started, and with it all components
    StopOrchestrator,
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

/// Local copy of a log stream of the orchestrator, fetched incrementally.
pub struct LogMirror {
    pub lines: VecDeque<String>,
    since: u64,
    instance_id: Option<u64>,
    capacity: usize,
    /// Prefix every line with the component name
    tagged: bool,
}

impl LogMirror {
    fn new(capacity: usize, tagged: bool) -> Self {
        Self {
            lines: VecDeque::with_capacity(capacity),
            since: 0,
            instance_id: None,
            capacity,
            tagged,
        }
    }

    fn clear(&mut self) {
        self.lines.clear();
        self.since = 0;
        self.instance_id = None;
    }

    fn push(&mut self, line: String) {
        if self.lines.len() >= self.capacity {
            self.lines.pop_front();
        }
        self.lines.push_back(line);
    }

    fn apply(&mut self, logs: Logs) {
        if self.instance_id.is_some_and(|id| id != logs.instance_id) {
            // The orchestrator was restarted and its sequence numbers start over, so this
            // answer is relative to a stale position. Start over with the next fetch.
            self.clear();
            self.push("[teamplayer] orchestrator restarted".to_string());
            self.instance_id = Some(logs.instance_id);
            return;
        }
        self.instance_id = Some(logs.instance_id);
        if logs.truncated {
            self.push("[teamplayer] … older lines were dropped by the orchestrator …".to_string());
        }
        for line in logs.lines {
            let err = if line.stream == LogStream::Stderr {
                "[ERR] "
            } else {
                ""
            };
            let text = if self.tagged {
                format!("[{}] {err}{}", line.component_name, line.text)
            } else {
                format!("{err}{}", line.text)
            };
            self.push(text);
            self.since = line.seq;
        }
    }
}

// ─── App ──────────────────────────────────────────────────────────────────────

pub struct App {
    pub client: Client,
    /// The orchestrator, if this TUI started it
    pub orchestrator: Option<OwnedOrchestrator>,
    /// Why the orchestrator started by this TUI is gone
    pub orchestrator_failure: Option<String>,
    /// Set when the TUI should exit
    pub exit: Option<ExitMode>,
    /// Latest status of the orchestrator, None until it was reached once
    pub status: Option<Status>,
    /// Why the orchestrator cannot be reached right now
    pub connection_error: Option<String>,
    /// Why the last action was refused
    pub action_error: Option<String>,
    pub screen: Screen,
    pub config_focus: ConfigFocus,
    pub config_fieldname_input: String,
    pub config_dsd_input: String,
    pub runtime_selected: usize,
    pub tick_count: u64,
    pub log_scroll: usize,
    pub log_follow: bool,
    /// Viewport height written by the draw function so scroll_keys can clamp correctly.
    pub log_viewport_height: std::cell::Cell<usize>,
    pub all_logs: LogMirror,
    /// Logs of the component shown on the Logs screen
    pub component_logs: LogMirror,
    component_logs_key: Option<String>,
    pub show_log_panel: bool,
    pub confirm_action: Option<ConfirmAction>,
}

impl App {
    pub fn new(client: Client) -> Self {
        App {
            client,
            orchestrator: None,
            orchestrator_failure: None,
            exit: None,
            status: None,
            connection_error: None,
            action_error: None,
            screen: Screen::Config,
            config_focus: ConfigFocus::Zenoh,
            config_fieldname_input: String::new(),
            config_dsd_input: String::new(),
            runtime_selected: 0,
            tick_count: 0,
            log_scroll: 0,
            log_follow: true,
            log_viewport_height: std::cell::Cell::new(0),
            all_logs: LogMirror::new(ALL_LOGS_CAPACITY, true),
            component_logs: LogMirror::new(COMPONENT_LOGS_CAPACITY, false),
            component_logs_key: None,
            show_log_panel: true,
            confirm_action: None,
        }
    }

    pub fn components(&self) -> &[ComponentInfo] {
        self.status
            .as_ref()
            .map(|s| s.components.as_slice())
            .unwrap_or_default()
    }

    pub fn sim(&self) -> bool {
        self.status.as_ref().is_some_and(|s| s.config.sim)
    }

    fn set_status(&mut self, status: Status) {
        if self.status.is_none() {
            // First contact: take over the orchestrator's config and jump straight to the
            // runtime view if it is already running something.
            self.config_fieldname_input = status.config.fieldname.clone();
            self.config_dsd_input = status.config.dsd_file.clone();
            if status.components.iter().any(|c| c.state.is_alive()) {
                self.screen = Screen::Runtime;
            }
        }
        self.status = Some(status);
    }

    /// Quit right away, or ask first if the TUI owns the orchestrator.
    pub fn request_quit(&mut self) {
        if self.orchestrator.is_some() {
            self.confirm_action = Some(ConfirmAction::Quit);
        } else {
            self.exit = Some(ExitMode::Detach);
        }
    }

    /// Fetch status and logs from the orchestrator.
    pub async fn refresh(&mut self) {
        if let Some(orchestrator) = &mut self.orchestrator {
            if let Some(status) = orchestrator.try_exited() {
                self.orchestrator_failure = Some(format!(
                    "the orchestrator started by teamplayer exited ({status}), see {}",
                    orchestrator.log_path.display()
                ));
                self.orchestrator = None;
            }
        }
        match self.client.status().await {
            Ok(status) => {
                self.set_status(status);
                self.connection_error = None;
            }
            Err(e) => {
                self.connection_error = Some(format!("{e:#}"));
                return;
            }
        }
        match self.client.logs(self.all_logs.since, None).await {
            Ok(logs) => self.all_logs.apply(logs),
            Err(e) => self.connection_error = Some(format!("{e:#}")),
        }
        if let Screen::Logs(idx) = self.screen {
            let Some(key) = self.components().get(idx).map(|c| c.key.clone()) else {
                return;
            };
            if self.component_logs_key.as_ref() != Some(&key) {
                self.component_logs.clear();
                self.component_logs_key = Some(key.clone());
            }
            match self
                .client
                .logs(self.component_logs.since, Some(&key))
                .await
            {
                Ok(logs) => self.component_logs.apply(logs),
                Err(e) => self.connection_error = Some(format!("{e:#}")),
            }
        }
    }

    /// Take over the status returned by an action, or remember why it failed.
    pub fn apply_action(&mut self, result: anyhow::Result<Status>) {
        match result {
            Ok(status) => {
                self.set_status(status);
                self.action_error = None;
            }
            Err(e) => self.action_error = Some(format!("{e:#}")),
        }
    }

    pub fn toggleable_indices(&self) -> Vec<usize> {
        let sim = self.sim();
        self.components()
            .iter()
            .enumerate()
            .filter(|(_, c)| !c.infrastructure && !c.sim_component && !(c.hardware_only && sim))
            .map(|(i, _)| i)
            .collect()
    }

    pub fn runtime_visible_indices(&self) -> Vec<usize> {
        let sim = self.sim();
        self.components()
            .iter()
            .enumerate()
            .filter(|(_, c)| {
                if c.sim_component {
                    return sim;
                }
                if c.hardware_only && sim {
                    return false;
                }
                c.enabled
            })
            .map(|(i, _)| i)
            .collect()
    }

    pub fn zenoh(&self) -> Option<&ComponentInfo> {
        self.components().iter().find(|c| c.key == "zenoh")
    }

    /// All focusable config items in reading order: flags row, left column, right column, Start.
    pub fn config_focus_order(&self) -> Vec<ConfigFocus> {
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
        let left_count = n.div_ceil(2);
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
        let left_count = self.toggleable_indices().len().div_ceil(2);
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
}

#[cfg(test)]
pub mod tests {
    use super::*;
    use bitbots_orchestrate::api::{Config, LogLine, ProcState};

    fn component(key: &str, infrastructure: bool) -> ComponentInfo {
        ComponentInfo {
            key: key.to_string(),
            name: key.to_string(),
            enabled: true,
            state: ProcState::Idle,
            pids: Vec::new(),
            exit_code: None,
            infrastructure,
            hardware_only: false,
            sim_component: false,
        }
    }

    /// An app that never reaches an orchestrator but has a known status.
    pub fn offline_app() -> App {
        let mut app = App::new(Client::new("/nonexistent/teamplayer-test.sock".into()));
        let mut components = vec![component("zenoh", true)];
        components.extend((0..5).map(|i| component(&format!("c{i}"), false)));
        app.set_status(Status {
            config: Config {
                sim: false,
                fieldname: "field".into(),
                dsd_file: "main.dsd".into(),
            },
            components,
        });
        app
    }

    #[test]
    fn first_status_initializes_inputs_and_screen() {
        let app = offline_app();
        assert_eq!(app.config_fieldname_input, "field");
        assert!(app.screen == Screen::Config);

        let mut app = App::new(Client::new("/nonexistent".into()));
        let mut running = component("vision", false);
        running.state = ProcState::Running;
        app.set_status(Status {
            config: offline_app().status.unwrap().config,
            components: vec![running],
        });
        assert!(app.screen == Screen::Runtime);
    }

    #[test]
    fn down_and_up_visit_every_component_and_are_inverse() {
        let mut app = offline_app();
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
        let mut app = offline_app();
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
        let mut app = offline_app();
        let left_count = app.toggleable_indices().len().div_ceil(2);
        app.config_focus = ConfigFocus::Component(0);
        app.config_focus_right();
        assert_eq!(app.config_focus, ConfigFocus::Component(left_count));
        app.config_focus_left();
        assert_eq!(app.config_focus, ConfigFocus::Component(0));
    }

    fn line(seq: u64, text: &str) -> LogLine {
        LogLine {
            seq,
            component: "vision".into(),
            component_name: "Vision".into(),
            stream: LogStream::Stderr,
            text: text.into(),
        }
    }

    #[test]
    fn log_mirror_follows_sequence_and_detects_restart() {
        let mut mirror = LogMirror::new(10, true);
        mirror.apply(Logs {
            lines: vec![line(1, "a"), line(2, "b")],
            truncated: false,
            latest_seq: 2,
            instance_id: 7,
        });
        assert_eq!(mirror.since, 2);
        assert_eq!(mirror.lines.back().unwrap(), "[Vision] [ERR] b");

        mirror.apply(Logs {
            lines: vec![line(1, "new")],
            truncated: false,
            latest_seq: 1,
            instance_id: 8,
        });
        assert_eq!(mirror.since, 0, "refetch from the start after a restart");
        assert_eq!(mirror.lines.len(), 1);
        assert!(mirror.lines[0].contains("restarted"));
    }
}
