use std::collections::{HashMap, VecDeque};

use bitbots_compose_api::{
    api::{
        ComponentInfo, LogStream, Logs, ParamType, ParameterInfo, ProcState, SetParameters, Status,
        Value,
    },
    client::Client,
};

use crate::spawn::OwnedComposeServer;

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
    /// Quit while owning the compose server: stop it or leave it running
    Quit,
}

#[derive(Debug, PartialEq)]
pub enum ExitMode {
    /// Leave the compose server and its components running
    Detach,
    /// Stop the compose server this TUI started, and with it all components
    StopComposeServer,
}

#[derive(Clone, PartialEq, Debug)]
pub enum ConfigFocus {
    Profile,
    /// Index into the parameters of the status
    Param(usize),
    /// Index into [`App::toggleable_indices`]
    Component(usize),
    Start,
}

/// Local copy of a log stream of the compose server, fetched incrementally.
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
            // The compose server was restarted and its sequence numbers start over, so this
            // answer is relative to a stale position. Start over with the next fetch.
            self.clear();
            self.push("[tui] compose server restarted".to_string());
            self.instance_id = Some(logs.instance_id);
            return;
        }
        self.instance_id = Some(logs.instance_id);
        if logs.truncated {
            self.push("[tui] … older lines were dropped by the compose server …".to_string());
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
    /// The compose server, if this TUI started it
    pub compose_server: Option<OwnedComposeServer>,
    /// Why the compose server started by this TUI is gone
    pub compose_server_failure: Option<String>,
    /// Set when the TUI should exit
    pub exit: Option<ExitMode>,
    /// Latest status of the compose server, None until it was reached once
    pub status: Option<Status>,
    /// Why the compose server cannot be reached right now
    pub connection_error: Option<String>,
    /// Why the last action was refused
    pub action_error: Option<String>,
    pub screen: Screen,
    pub config_focus: ConfigFocus,
    /// Edit buffers of the string parameters, sent to the compose server on launch
    pub param_inputs: HashMap<String, String>,
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
            compose_server: None,
            compose_server_failure: None,
            exit: None,
            status: None,
            connection_error: None,
            action_error: None,
            screen: Screen::Config,
            config_focus: ConfigFocus::Profile,
            param_inputs: HashMap::new(),
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

    pub fn parameters(&self) -> &[ParameterInfo] {
        self.status
            .as_ref()
            .map(|s| s.parameters.as_slice())
            .unwrap_or_default()
    }

    fn set_status(&mut self, status: Status) {
        let first = self.status.is_none();
        if first && status.components.iter().any(|c| c.state.is_started()) {
            // Jump straight to the runtime view if the compose server already runs something.
            self.screen = Screen::Runtime;
        }
        self.status = Some(status);
        if first {
            self.sync_inputs();
        }
    }

    /// Reset the edit buffers to the compose server's parameter values.
    pub fn sync_inputs(&mut self) {
        self.param_inputs = self
            .parameters()
            .iter()
            .filter_map(|p| match &p.value {
                Value::String(v) => Some((p.name.clone(), v.clone())),
                Value::Bool(_) => None,
            })
            .collect();
    }

    /// The edited values of all free text parameters.
    pub fn edited_parameters(&self) -> SetParameters {
        self.parameters()
            .iter()
            .filter(|p| p.param_type == ParamType::String && p.choices.is_none())
            .filter_map(|p| {
                let input = self.param_inputs.get(&p.name)?;
                Some((p.name.clone(), Value::String(input.clone())))
            })
            .collect()
    }

    /// Description of the focused config item, if it has one.
    pub fn focused_description(&self) -> Option<String> {
        match self.config_focus {
            ConfigFocus::Profile => {
                let status = self.status.as_ref()?;
                let current = status.profile.as_ref()?;
                let profile = status.profiles.iter().find(|p| &p.name == current)?;
                profile.description.clone()
            }
            ConfigFocus::Param(i) => self.parameters().get(i).map(|p| p.description.clone()),
            ConfigFocus::Component(fi) => {
                let idx = *self.toggleable_indices().get(fi)?;
                self.components()[idx].description.clone()
            }
            ConfigFocus::Start => None,
        }
    }

    /// Ask whether to stop the compose server or leave it running, or quit right away if there is
    /// no compose server to ask about.
    pub fn request_quit(&mut self) {
        if self.status.is_some() && self.connection_error.is_none() {
            self.confirm_action = Some(ConfirmAction::Quit);
        } else {
            self.exit = Some(ExitMode::Detach);
        }
    }

    /// Fetch status and logs from the compose server.
    pub async fn refresh(&mut self) {
        if let Some(compose_server) = &mut self.compose_server {
            if let Some(status) = compose_server.try_exited() {
                self.compose_server_failure = Some(format!(
                    "the compose server started by the TUI exited ({status}), see {}",
                    compose_server.log_path.display()
                ));
                self.compose_server = None;
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
        self.components()
            .iter()
            .enumerate()
            .filter(|(_, c)| c.toggleable())
            .map(|(i, _)| i)
            .collect()
    }

    /// Components that are selected to run or have run.
    pub fn runtime_visible_indices(&self) -> Vec<usize> {
        self.components()
            .iter()
            .enumerate()
            .filter(|(_, c)| c.active || c.state != ProcState::Idle)
            .map(|(i, _)| i)
            .collect()
    }

    /// Items of the top row: the profile, then all parameters.
    fn row_items(&self) -> Vec<ConfigFocus> {
        [ConfigFocus::Profile]
            .into_iter()
            .chain((0..self.parameters().len()).map(ConfigFocus::Param))
            .collect()
    }

    /// All focusable config items in reading order: top row, left column, right column, Start.
    pub fn config_focus_order(&self) -> Vec<ConfigFocus> {
        let n = self.toggleable_indices().len();
        self.row_items()
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

    /// Down: top row → first component; then down the left column, continuing at the top
    /// of the right column; bottom of the right column → Start; Start wraps to the top row.
    pub fn config_focus_next(&mut self) {
        let n = self.toggleable_indices().len();
        self.config_focus = match self.config_focus.clone() {
            ConfigFocus::Profile | ConfigFocus::Param(_) => {
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
            ConfigFocus::Start => ConfigFocus::Profile,
        };
    }

    /// Up: exact inverse of Down, except that the top row wraps to Start.
    pub fn config_focus_prev(&mut self) {
        let n = self.toggleable_indices().len();
        self.config_focus = match self.config_focus.clone() {
            ConfigFocus::Profile | ConfigFocus::Param(_) => ConfigFocus::Start,
            ConfigFocus::Component(0) => ConfigFocus::Profile,
            ConfigFocus::Component(i) => ConfigFocus::Component(i - 1),
            ConfigFocus::Start => {
                if n > 0 {
                    ConfigFocus::Component(n - 1)
                } else {
                    ConfigFocus::Profile
                }
            }
        };
    }

    pub fn config_focus_right(&mut self) {
        let n = self.toggleable_indices().len();
        let left_count = n.div_ceil(2);
        let row = self.row_items();
        match self.config_focus.clone() {
            ConfigFocus::Component(i) if i + left_count < n => {
                self.config_focus = ConfigFocus::Component(i + left_count);
            }
            focus @ (ConfigFocus::Profile | ConfigFocus::Param(_)) => {
                let pos = row.iter().position(|f| *f == focus).unwrap_or(0);
                if let Some(next) = row.get(pos + 1) {
                    self.config_focus = next.clone();
                }
            }
            _ => {}
        }
    }

    pub fn config_focus_left(&mut self) {
        let left_count = self.toggleable_indices().len().div_ceil(2);
        let row = self.row_items();
        match self.config_focus.clone() {
            ConfigFocus::Component(i) if i >= left_count => {
                self.config_focus = ConfigFocus::Component(i - left_count);
            }
            focus @ (ConfigFocus::Profile | ConfigFocus::Param(_)) => {
                let pos = row.iter().position(|f| *f == focus).unwrap_or(0);
                if pos > 0 {
                    self.config_focus = row[pos - 1].clone();
                }
            }
            _ => {}
        }
    }
}

#[cfg(test)]
pub mod tests {
    use super::*;
    use bitbots_compose_api::api::{LogLine, ProfileInfo};

    fn component(key: &str, locked: bool) -> ComponentInfo {
        ComponentInfo {
            key: key.to_string(),
            name: key.to_string(),
            description: Some(format!("about {key}")),
            enabled: true,
            locked,
            fallback_for: None,
            requires: Vec::new(),
            available: true,
            active: true,
            state: ProcState::Idle,
            pids: Vec::new(),
            exit_code: None,
        }
    }

    fn status(components: Vec<ComponentInfo>) -> Status {
        Status {
            parameters: vec![
                ParameterInfo {
                    name: "sim".into(),
                    param_type: ParamType::Bool,
                    description: "Simulation".into(),
                    value: Value::Bool(false),
                    choices: None,
                },
                ParameterInfo {
                    name: "field".into(),
                    param_type: ParamType::String,
                    description: "Field".into(),
                    value: Value::String("lab".into()),
                    choices: None,
                },
            ],
            profiles: vec![ProfileInfo {
                name: "sim".into(),
                description: None,
            }],
            profile: None,
            problems: Vec::new(),
            components,
            global_parameters: Default::default(),
        }
    }

    /// An app that never reaches a compose server but has a known status.
    pub fn offline_app() -> App {
        let mut app = App::new(Client::new("/nonexistent/teamplayer-test.sock".into()));
        let mut components = vec![component("base", true)];
        components.extend((0..5).map(|i| component(&format!("c{i}"), false)));
        app.set_status(status(components));
        app
    }

    #[test]
    fn first_status_initializes_inputs_and_screen() {
        let app = offline_app();
        assert_eq!(app.param_inputs["field"], "lab");
        assert!(!app.param_inputs.contains_key("sim"));
        assert!(app.screen == Screen::Config);
        assert_eq!(
            app.edited_parameters(),
            SetParameters::from([("field".to_string(), Value::String("lab".into()))])
        );

        let mut app = App::new(Client::new("/nonexistent".into()));
        let mut running = component("vision", false);
        running.state = ProcState::Running;
        app.set_status(status(vec![running]));
        assert!(app.screen == Screen::Runtime);
    }

    #[test]
    fn locked_components_are_not_toggleable() {
        let app = offline_app();
        assert_eq!(app.toggleable_indices(), [1, 2, 3, 4, 5]);
        app.components()[1..]
            .iter()
            .for_each(|c| assert!(c.toggleable()));
    }

    #[test]
    fn down_and_up_visit_every_component_and_are_inverse() {
        let mut app = offline_app();
        let n = app.toggleable_indices().len();
        assert!(n >= 2, "test needs both config columns populated");

        app.config_focus = ConfigFocus::Profile;
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
        assert_eq!(app.config_focus, ConfigFocus::Profile);
    }

    #[test]
    fn tab_cycles_through_all_items() {
        let mut app = offline_app();
        let order = app.config_focus_order();
        assert_eq!(
            order[..3],
            [
                ConfigFocus::Profile,
                ConfigFocus::Param(0),
                ConfigFocus::Param(1)
            ]
        );
        app.config_focus = order[0].clone();
        for expected in order.iter().skip(1).chain(order.first()) {
            app.config_focus_tab();
            assert_eq!(&app.config_focus, expected);
        }
        app.config_focus_backtab();
        assert_eq!(&app.config_focus, order.last().unwrap());
    }

    #[test]
    fn left_right_move_along_the_row_and_between_columns() {
        let mut app = offline_app();
        app.config_focus = ConfigFocus::Profile;
        app.config_focus_right();
        app.config_focus_right();
        assert_eq!(app.config_focus, ConfigFocus::Param(1));
        app.config_focus_right();
        assert_eq!(
            app.config_focus,
            ConfigFocus::Param(1),
            "stays at the row end"
        );
        app.config_focus_left();
        assert_eq!(app.config_focus, ConfigFocus::Param(0));

        let left_count = app.toggleable_indices().len().div_ceil(2);
        app.config_focus = ConfigFocus::Component(0);
        app.config_focus_right();
        assert_eq!(app.config_focus, ConfigFocus::Component(left_count));
        app.config_focus_left();
        assert_eq!(app.config_focus, ConfigFocus::Component(0));
    }

    #[test]
    fn focused_description_comes_from_the_status() {
        let mut app = offline_app();
        app.config_focus = ConfigFocus::Param(1);
        assert_eq!(app.focused_description().as_deref(), Some("Field"));
        app.config_focus = ConfigFocus::Component(0);
        assert_eq!(app.focused_description().as_deref(), Some("about c0"));
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
