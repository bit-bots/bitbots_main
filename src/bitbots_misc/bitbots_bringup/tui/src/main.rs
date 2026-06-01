use std::{
    collections::VecDeque,
    sync::{Arc, Mutex},
    time::Duration,
};

use anyhow::Result;
use crossterm::{
    event::{DisableMouseCapture, EnableMouseCapture, Event, EventStream, KeyCode, KeyModifiers},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use futures::StreamExt;
use ratatui::{
    backend::CrosstermBackend,
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Paragraph},
    Frame, Terminal,
};
use tokio::{
    io::{AsyncBufReadExt, BufReader},
    process::Command,
    sync::mpsc,
};

// ─── Types ────────────────────────────────────────────────────────────────────

#[derive(Clone, Debug, PartialEq)]
enum ProcState {
    Idle,
    Running,
    Stopping,
    Stopped,
    Crashed,
}

impl ProcState {
    fn label(&self) -> &'static str {
        match self {
            ProcState::Idle => "IDLE",
            ProcState::Running => "RUN ",
            ProcState::Stopping => "STOP",
            ProcState::Stopped => "DONE",
            ProcState::Crashed => "FAIL",
        }
    }
    fn style(&self) -> Style {
        match self {
            ProcState::Idle => Style::default().fg(Color::DarkGray),
            ProcState::Running => Style::default().fg(Color::Green),
            ProcState::Stopping => Style::default().fg(Color::Yellow),
            ProcState::Stopped => Style::default().fg(Color::Blue),
            ProcState::Crashed => Style::default().fg(Color::Red),
        }
    }
}

#[derive(Clone, Debug)]
struct GlobalParams {
    sim: bool,
    zenoh: bool,
    fieldname: String,
    dsd_file: String,
}

impl Default for GlobalParams {
    fn default() -> Self {
        Self {
            sim: false,
            zenoh: true,
            fieldname: String::new(),
            dsd_file: "main.dsd".to_string(),
        }
    }
}

struct ComponentDef {
    name: &'static str,
    key: &'static str,
    default_enabled: bool,
    infrastructure: bool,
    /// Component is only valid on real hardware — hidden/disabled when sim=true
    hardware_only: bool,
    cmds: fn(&GlobalParams) -> Vec<Vec<String>>,
}

struct ComponentState {
    def_idx: usize,
    enabled: bool,
    state: ProcState,
    logs: Arc<Mutex<VecDeque<String>>>,
    pids: Vec<u32>,
    task_handles: Vec<tokio::task::JoinHandle<()>>,
}

impl ComponentState {
    fn new(def_idx: usize, enabled: bool) -> Self {
        Self {
            def_idx,
            enabled,
            state: ProcState::Idle,
            logs: Arc::new(Mutex::new(VecDeque::with_capacity(2000))),
            pids: Vec::new(),
            task_handles: Vec::new(),
        }
    }
}

enum AppMsg {
    ProcessExited { key: String, code: i32 },
}

// ─── Component definitions ────────────────────────────────────────────────────

fn make_ros2_launch(pkg: &str, file: &str, extra: &[&str]) -> Vec<String> {
    let mut cmd = vec![
        "ros2".to_string(),
        "launch".to_string(),
        pkg.to_string(),
        file.to_string(),
    ];
    for e in extra {
        cmd.push(e.to_string());
    }
    cmd
}

fn make_ros2_run(pkg: &str, exec: &str, extra: &[&str]) -> Vec<String> {
    let mut cmd = vec![
        "ros2".to_string(),
        "run".to_string(),
        pkg.to_string(),
        exec.to_string(),
    ];
    for e in extra {
        cmd.push(e.to_string());
    }
    cmd
}

fn sim_arg(params: &GlobalParams) -> String {
    format!("sim:={}", params.sim)
}

static COMPONENT_DEFS: &[ComponentDef] = &[
    // ── Infrastructure (always started) ──
    ComponentDef {
        name: "Zenoh",
        key: "zenoh",
        default_enabled: true,
        infrastructure: true,
        hardware_only: false,
        cmds: |_p| vec![make_ros2_run("rmw_zenoh_cpp", "rmw_zenohd", &[])],
    },
    ComponentDef {
        name: "Param Blackboard",
        key: "blackboard",
        default_enabled: true,
        infrastructure: true,
        hardware_only: false,
        cmds: |_p| {
            vec![make_ros2_run(
                "parameter_blackboard",
                "parameter_blackboard",
                &["--ros-args", "-p", "use_sim_time:=false"],
            )]
        },
    },
    ComponentDef {
        name: "Robot Description",
        key: "robot_description",
        default_enabled: true,
        infrastructure: true,
        hardware_only: false,
        cmds: |p| {
            vec![make_ros2_launch(
                "bitbots_bringup",
                "robot_description.launch",
                &[&sim_arg(p)],
            )]
        },
    },
    ComponentDef {
        name: "Diagnostics",
        key: "diagnostics",
        default_enabled: true,
        infrastructure: true,
        hardware_only: false,
        cmds: |_p| vec![make_ros2_launch("bitbots_bringup", "diagnostics.launch", &[])],
    },
    ComponentDef {
        name: "Simulator",
        key: "simulator",
        default_enabled: false,
        infrastructure: true,
        hardware_only: false,
        cmds: |p| {
            vec![make_ros2_launch(
                "bitbots_bringup",
                "mujoco_simulation.launch.py",
                &[&sim_arg(p)],
            )]
        },
    },
    // ── User-toggleable ──
    ComponentDef {
        name: "Lowlevel",
        key: "lowlevel",
        default_enabled: true,
        infrastructure: false,
        hardware_only: true, // hardware interface — never in sim
        cmds: |_p| vec![make_ros2_launch("livelybot_bringup", "lowlevel.launch", &[])],
    },
    ComponentDef {
        name: "Motion",
        key: "motion",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        cmds: |p| {
            vec![make_ros2_launch(
                "bitbots_bringup",
                "motion.launch",
                &[&sim_arg(p)],
            )]
        },
    },
    ComponentDef {
        name: "Game Controller",
        key: "game_controller",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        cmds: |p| {
            vec![make_ros2_launch(
                "game_controller_hsl",
                "game_controller.launch",
                &[
                    &sim_arg(p),
                    "use_parameter_blackboard:=true",
                    "parameter_blackboard_name:=parameter_blackboard",
                    "team_id_param_name:=team_id",
                    "bot_id_param_name:=bot_id",
                ],
            )]
        },
    },
    ComponentDef {
        name: "Vision",
        key: "vision",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        cmds: |p| {
            vec![make_ros2_launch("bitbots_bringup", "vision.launch", &[&sim_arg(p)])]
        },
    },
    ComponentDef {
        name: "IPM",
        key: "ipm",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        cmds: |p| {
            vec![make_ros2_launch("bitbots_ipm", "ipm.launch", &[&sim_arg(p)])]
        },
    },
    ComponentDef {
        name: "Localization",
        key: "localization",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        cmds: |p| {
            vec![make_ros2_launch(
                "bitbots_localization",
                "localization.launch",
                &[&sim_arg(p)],
            )]
        },
    },
    ComponentDef {
        name: "Path Planning",
        key: "path_planning",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        cmds: |p| {
            vec![make_ros2_launch(
                "bitbots_path_planning",
                "path_planning.launch",
                &[&sim_arg(p)],
            )]
        },
    },
    ComponentDef {
        name: "Behavior",
        key: "behavior",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        cmds: |p| {
            vec![make_ros2_launch(
                "bitbots_body_behavior",
                "behavior.launch",
                &[&sim_arg(p), &format!("dsd_file:={}", p.dsd_file)],
            )]
        },
    },
    ComponentDef {
        name: "Team Comm",
        key: "teamcom",
        default_enabled: false,
        infrastructure: false,
        hardware_only: false,
        cmds: |p| {
            vec![make_ros2_launch(
                "bitbots_team_communication",
                "team_comm.launch",
                &[&sim_arg(p)],
            )]
        },
    },
    ComponentDef {
        name: "World Model",
        key: "world_model",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        cmds: |p| {
            vec![
                make_ros2_launch("bitbots_ball_filter", "ball_filter.launch", &[&sim_arg(p)]),
                make_ros2_launch("bitbots_robot_filter", "robot_filter.launch", &[&sim_arg(p)]),
            ]
        },
    },
    ComponentDef {
        name: "Whistle Det.",
        key: "whistle_detector",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        cmds: |_p| {
            vec![make_ros2_launch(
                "bitbots_whistle_detector",
                "whistle_detector.launch",
                &[],
            )]
        },
    },
    ComponentDef {
        name: "Audio",
        key: "audio",
        default_enabled: true,
        infrastructure: false,
        hardware_only: false,
        cmds: |_p| vec![make_ros2_launch("bitbots_bringup", "audio.launch", &[])],
    },
    ComponentDef {
        name: "TTS",
        key: "tts",
        default_enabled: false,
        infrastructure: false,
        hardware_only: false,
        cmds: |_p| vec![make_ros2_launch("bitbots_bringup", "tts.launch", &[])],
    },
    ComponentDef {
        name: "Monitoring",
        key: "monitoring",
        default_enabled: false,
        infrastructure: false,
        hardware_only: false,
        cmds: |_p| vec![make_ros2_launch("bitbots_bringup", "monitoring.launch", &[])],
    },
    ComponentDef {
        name: "Recording",
        key: "record",
        default_enabled: false,
        infrastructure: false,
        hardware_only: false,
        cmds: |p| {
            vec![make_ros2_launch(
                "bitbots_bringup",
                "record.launch",
                &[&format!("fieldname:={}", p.fieldname)],
            )]
        },
    },
];

// ─── Screens ──────────────────────────────────────────────────────────────────

#[derive(PartialEq, Clone)]
enum Screen {
    Config,
    Runtime,
    Logs(usize), // component index
}

// Config screen focus items
#[derive(Clone, PartialEq, Debug)]
enum ConfigFocus {
    Zenoh,
    Sim,
    Fieldname,
    DsdFile,
    Component(usize), // index into toggleable components list
    Start,
}

// ─── App state ────────────────────────────────────────────────────────────────

struct App {
    screen: Screen,
    params: GlobalParams,
    components: Vec<ComponentState>,
    config_focus: ConfigFocus,
    config_fieldname_input: String,
    config_dsd_input: String,
    runtime_selected: usize,
    tick_count: u64,
    msg_tx: mpsc::UnboundedSender<AppMsg>,
    msg_rx: mpsc::UnboundedReceiver<AppMsg>,
    log_scroll: usize,
}

impl App {
    fn new() -> Self {
        let (msg_tx, msg_rx) = mpsc::unbounded_channel();
        let mut components: Vec<ComponentState> = COMPONENT_DEFS
            .iter()
            .enumerate()
            .map(|(i, def)| ComponentState::new(i, def.default_enabled))
            .collect();

        // Simulator is only enabled when sim=true
        let sim_idx = COMPONENT_DEFS.iter().position(|d| d.key == "simulator").unwrap();
        components[sim_idx].enabled = false;

        App {
            screen: Screen::Config,
            params: GlobalParams::default(),
            components,
            config_focus: ConfigFocus::Zenoh,
            config_fieldname_input: String::new(),
            config_dsd_input: "main.dsd".to_string(),
            runtime_selected: 0,
            tick_count: 0,
            msg_tx,
            msg_rx,
            log_scroll: 0,
        }
    }

    fn toggleable_indices(&self) -> Vec<usize> {
        COMPONENT_DEFS
            .iter()
            .enumerate()
            .filter(|(_, d)| !d.infrastructure && !(d.hardware_only && self.params.sim))
            .map(|(i, _)| i)
            .collect()
    }

    fn config_focus_items(&self) -> Vec<ConfigFocus> {
        let mut items = vec![ConfigFocus::Zenoh, ConfigFocus::Sim, ConfigFocus::Fieldname, ConfigFocus::DsdFile];
        let toggleables = self.toggleable_indices();
        for i in 0..toggleables.len() {
            items.push(ConfigFocus::Component(i));
        }
        items.push(ConfigFocus::Start);
        items
    }

    fn config_focus_next(&mut self) {
        let items = self.config_focus_items();
        let pos = items.iter().position(|f| f == &self.config_focus).unwrap_or(0);
        self.config_focus = items[(pos + 1) % items.len()].clone();
    }

    fn config_focus_prev(&mut self) {
        let items = self.config_focus_items();
        let pos = items.iter().position(|f| f == &self.config_focus).unwrap_or(0);
        self.config_focus = items[(pos + items.len() - 1) % items.len()].clone();
    }

    fn runtime_visible_indices(&self) -> Vec<usize> {
        COMPONENT_DEFS
            .iter()
            .enumerate()
            .filter(|(i, d)| {
                // Simulator row only appears when sim=true
                if d.key == "simulator" {
                    return self.params.sim;
                }
                // Hardware-only components are never shown in sim
                if d.hardware_only && self.params.sim {
                    return false;
                }
                self.components[*i].enabled
            })
            .map(|(i, _)| i)
            .collect()
    }

    async fn start_component(&mut self, comp_idx: usize) {
        let def = &COMPONENT_DEFS[comp_idx];
        let cmds = (def.cmds)(&self.params);
        let key = def.key.to_string();

        // Extract what we need before the loop so we don't hold &mut self.components
        let logs = self.components[comp_idx].logs.clone();
        self.components[comp_idx].state = ProcState::Running;
        self.components[comp_idx].pids.clear();
        self.components[comp_idx].task_handles.clear();

        for cmd_args in cmds {
            if cmd_args.is_empty() {
                continue;
            }
            let tx = self.msg_tx.clone();
            let logs = logs.clone();
            let key = key.clone();

            let mut child = match Command::new(&cmd_args[0])
                .args(&cmd_args[1..])
                .stdout(std::process::Stdio::piped())
                .stderr(std::process::Stdio::piped())
                // New process group so we can kill the entire ros2 launch tree
                .process_group(0)
                .kill_on_drop(true)
                .spawn()
            {
                Ok(c) => c,
                Err(e) => {
                    let mut l = logs.lock().unwrap();
                    l.push_back(format!("[ERROR] spawn failed: {e}"));
                    drop(l);
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
                // Interleave stdout+stderr into log buffer
                if let Some(mut reader) = stdout {
                    let mut line = String::new();
                    while let Ok(n) = reader.read_line(&mut line).await {
                        if n == 0 { break; }
                        let trimmed = line.trim_end_matches('\n').to_string();
                        let mut l = logs.lock().unwrap();
                        if l.len() >= 2000 { l.pop_front(); }
                        l.push_back(trimmed);
                        line.clear();
                    }
                }
                if let Some(mut reader) = stderr {
                    let mut line = String::new();
                    while let Ok(n) = reader.read_line(&mut line).await {
                        if n == 0 { break; }
                        let trimmed = format!("[ERR] {}", line.trim_end_matches('\n'));
                        let mut l = logs.lock().unwrap();
                        if l.len() >= 2000 { l.pop_front(); }
                        l.push_back(trimmed);
                        line.clear();
                    }
                }
                let code = child.wait().await.map(|s| s.code().unwrap_or(-1)).unwrap_or(-1);
                let _ = tx.send(AppMsg::ProcessExited { key, code });
            });
            self.components[comp_idx].task_handles.push(handle);
        }
    }

    async fn stop_component(&mut self, comp_idx: usize) {
        let comp = &mut self.components[comp_idx];
        if comp.state != ProcState::Running {
            return;
        }
        comp.state = ProcState::Stopping;
        let pids = comp.pids.clone();
        for &pid in &pids {
            unsafe {
                // Negative PID = send to entire process group (kills ros2 launch children too)
                libc::kill(-(pid as libc::pid_t), libc::SIGTERM);
            }
        }
        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_secs(5)).await;
            for pid in pids {
                unsafe {
                    libc::kill(-(pid as libc::pid_t), libc::SIGKILL);
                }
            }
        });
    }

    async fn stop_all(&mut self) {
        let indices: Vec<usize> = (0..self.components.len()).collect();
        for i in indices {
            if self.components[i].state == ProcState::Running {
                self.stop_component(i).await;
            }
        }
    }

    async fn start_all_enabled(&mut self) {
        let enabled: Vec<usize> = self.components
            .iter()
            .enumerate()
            .filter(|(_, c)| c.enabled && c.state == ProcState::Idle)
            .map(|(i, _)| i)
            .collect();
        for i in enabled {
            self.start_component(i).await;
        }
    }

    fn drain_messages(&mut self) {
        while let Ok(msg) = self.msg_rx.try_recv() {
            match msg {
                AppMsg::ProcessExited { key, code } => {
                    if let Some(comp) = self.components.iter_mut().find(|c| {
                        COMPONENT_DEFS[c.def_idx].key == key
                    }) {
                        if comp.state == ProcState::Stopping || comp.state == ProcState::Running {
                            comp.state = if code == 0 {
                                ProcState::Stopped
                            } else {
                                ProcState::Crashed
                            };
                            comp.pids.clear();
                        }
                    }
                }
            }
        }
    }
}

// ─── Rendering ────────────────────────────────────────────────────────────────

const BANNER: &str = r#"
  ██████╗ ██╗████████╗    ██████╗  ██████╗ ████████╗███████╗
  ██╔══██╗██║╚══██╔══╝    ██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝
  ██████╔╝██║   ██║       ██████╔╝██║   ██║   ██║   ███████╗
  ██╔══██╗██║   ██║       ██╔══██╗██║   ██║   ██║   ╚════██║
  ██████╔╝██║   ██║       ██████╔╝╚██████╔╝   ██║   ███████║
  ╚═════╝ ╚═╝   ╚═╝       ╚═════╝  ╚═════╝    ╚═╝   ╚══════╝
                     Teamplayer Launcher
"#;

fn spinner_char(tick: u64) -> char {
    const FRAMES: &[char] = &['⠋', '⠙', '⠹', '⠸', '⠼', '⠴', '⠦', '⠧', '⠇', '⠏'];
    FRAMES[(tick as usize) % FRAMES.len()]
}

fn draw(f: &mut Frame, app: &App) {
    match &app.screen {
        Screen::Config => draw_config(f, app),
        Screen::Runtime => draw_runtime(f, app),
        Screen::Logs(idx) => draw_logs(f, app, *idx),
    }
}

fn draw_config(f: &mut Frame, app: &App) {
    let area = f.size();

    let banner_lines = BANNER.lines().count() as u16;

    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(banner_lines),
            Constraint::Length(3), // global flags bar
            Constraint::Min(0),    // component grid
            Constraint::Length(3), // Start/Continue button
            Constraint::Length(1), // footer
        ])
        .split(area);

    let banner = Paragraph::new(BANNER)
        .style(Style::default().fg(Color::Cyan))
        .alignment(Alignment::Center);
    f.render_widget(banner, chunks[0]);

    draw_config_flags(f, app, chunks[1]);
    draw_config_components(f, app, chunks[2]);
    draw_start_button(f, app, chunks[3]);

    let footer = Paragraph::new(
        " Tab/↑↓: navigate  Space/Enter: toggle/select  q: quit",
    )
    .style(Style::default().fg(Color::DarkGray));
    f.render_widget(footer, chunks[4]);
}

fn draw_start_button(f: &mut Frame, app: &App, area: Rect) {
    let focused = app.config_focus == ConfigFocus::Start;
    let has_running = app.components.iter().any(|c| c.state == ProcState::Running);

    let label = if has_running { "  Continue →  " } else { "  Launch →  " };

    let (border_style, text_style) = if focused {
        (
            Style::default().fg(Color::Yellow),
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD),
        )
    } else {
        (
            Style::default().fg(Color::Green),
            Style::default().fg(Color::Green).add_modifier(Modifier::BOLD),
        )
    };

    let button = Paragraph::new(label)
        .block(Block::default().borders(Borders::ALL).border_style(border_style))
        .style(text_style)
        .alignment(Alignment::Center);
    f.render_widget(button, area);
}

fn draw_config_flags(f: &mut Frame, app: &App, area: Rect) {
    let chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Length(18), // Zenoh
            Constraint::Length(18), // Sim
            Constraint::Min(20),    // Fieldname
            Constraint::Min(20),    // DSD file
        ])
        .split(area);

    let zenoh_focused = app.config_focus == ConfigFocus::Zenoh;
    let sim_focused = app.config_focus == ConfigFocus::Sim;
    let field_focused = app.config_focus == ConfigFocus::Fieldname;
    let dsd_focused = app.config_focus == ConfigFocus::DsdFile;

    let zenoh_style = if zenoh_focused {
        Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)
    } else {
        Style::default().fg(Color::White)
    };
    let zenoh_text = if app.params.zenoh { "[✓] Zenoh" } else { "[ ] Zenoh" };
    let zenoh_p = Paragraph::new(zenoh_text)
        .block(Block::default().borders(Borders::ALL).border_style(if zenoh_focused {
            Style::default().fg(Color::Yellow)
        } else {
            Style::default()
        }))
        .style(zenoh_style);
    f.render_widget(zenoh_p, chunks[0]);

    let sim_style = if sim_focused {
        Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)
    } else {
        Style::default().fg(Color::White)
    };
    let sim_text = if app.params.sim { "[✓] Simulator" } else { "[ ] Simulator" };
    let sim_p = Paragraph::new(sim_text)
        .block(Block::default().borders(Borders::ALL).border_style(if sim_focused {
            Style::default().fg(Color::Yellow)
        } else {
            Style::default()
        }))
        .style(sim_style);
    f.render_widget(sim_p, chunks[1]);

    let field_p = Paragraph::new(app.config_fieldname_input.as_str())
        .block(Block::default().borders(Borders::ALL).title("Fieldname").border_style(if field_focused {
            Style::default().fg(Color::Yellow)
        } else {
            Style::default()
        }));
    f.render_widget(field_p, chunks[2]);

    let dsd_p = Paragraph::new(app.config_dsd_input.as_str())
        .block(Block::default().borders(Borders::ALL).title("DSD File").border_style(if dsd_focused {
            Style::default().fg(Color::Yellow)
        } else {
            Style::default()
        }));
    f.render_widget(dsd_p, chunks[3]);
}

fn draw_config_components(f: &mut Frame, app: &App, area: Rect) {
    let toggleables = app.toggleable_indices();

    // Split into two columns
    let col_chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(area);

    let left_count = (toggleables.len() + 1) / 2;
    let (left_indices, right_indices) = toggleables.split_at(left_count);

    draw_component_column(f, app, col_chunks[0], left_indices, 0);
    draw_component_column(f, app, col_chunks[1], right_indices, left_indices.len());
}

fn draw_component_column(
    f: &mut Frame,
    app: &App,
    area: Rect,
    comp_indices: &[usize],
    focus_offset: usize,
) {
    let rows: Vec<Constraint> = comp_indices
        .iter()
        .map(|_| Constraint::Length(1))
        .collect();
    if rows.is_empty() {
        return;
    }
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints(rows)
        .split(area);

    for (local_i, &comp_idx) in comp_indices.iter().enumerate() {
        let focus_i = focus_offset + local_i;
        let focused = app.config_focus == ConfigFocus::Component(focus_i);
        let comp = &app.components[comp_idx];
        let def = &COMPONENT_DEFS[comp_idx];

        let check = if comp.enabled { "✓" } else { " " };
        let text = format!("[{}] {}", check, def.name);

        let style = if focused {
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD)
        } else if comp.enabled {
            Style::default().fg(Color::White)
        } else {
            Style::default().fg(Color::DarkGray)
        };

        let p = Paragraph::new(text).style(style);
        if local_i < chunks.len() {
            f.render_widget(p, chunks[local_i]);
        }
    }

    // Start button at bottom of right column
    // (handled separately in draw_config)
}

fn draw_runtime(f: &mut Frame, app: &App) {
    let area = f.size();

    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3), // header
            Constraint::Min(0),    // component list
            Constraint::Length(1), // footer
        ])
        .split(area);

    // Header
    draw_runtime_header(f, app, chunks[0]);

    // Component list
    draw_runtime_list(f, app, chunks[1]);

    // Footer
    let footer = Paragraph::new(
        " ↑↓: select  s: start/stop  r: restart  l: logs  a: start all  x: stop all  q: quit",
    )
    .style(Style::default().fg(Color::DarkGray));
    f.render_widget(footer, chunks[2]);
}

fn draw_runtime_header(f: &mut Frame, app: &App, area: Rect) {
    let running = app.components.iter().filter(|c| c.state == ProcState::Running).count();
    let total_enabled = app.components.iter().filter(|c| c.enabled).count();

    let title = format!(" Bit-Bots Teamplayer  [{running}/{total_enabled} running]");
    let sim_badge = if app.params.sim { "  SIM " } else { "" };
    let text = format!("{title}{sim_badge}");

    let p = Paragraph::new(text)
        .block(Block::default().borders(Borders::ALL))
        .style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD));
    f.render_widget(p, area);
}

fn draw_runtime_list(f: &mut Frame, app: &App, area: Rect) {
    let visible = app.runtime_visible_indices();
    if visible.is_empty() {
        let p = Paragraph::new("No components enabled.").alignment(Alignment::Center);
        f.render_widget(p, area);
        return;
    }

    let row_height = 1u16;
    let rows: Vec<Constraint> = visible.iter().map(|_| Constraint::Length(row_height)).collect();
    let row_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints(rows)
        .split(area);

    for (vis_i, &comp_idx) in visible.iter().enumerate() {
        let selected = app.runtime_selected == vis_i;
        let comp = &app.components[comp_idx];
        let def = &COMPONENT_DEFS[comp_idx];

        if vis_i < row_chunks.len() {
            draw_runtime_row(f, app, row_chunks[vis_i], comp, def, selected);
        }
    }
}

fn draw_runtime_row(
    f: &mut Frame,
    app: &App,
    area: Rect,
    comp: &ComponentState,
    def: &ComponentDef,
    selected: bool,
) {
    let row_style = if selected {
        Style::default().bg(Color::DarkGray)
    } else {
        Style::default()
    };

    let spinner = match comp.state {
        ProcState::Running => format!("{} ", spinner_char(app.tick_count)),
        ProcState::Stopping => "… ".to_string(),
        _ => "  ".to_string(),
    };

    let state_label = comp.state.label();
    let state_style = comp.state.style();

    let infra_marker = if def.infrastructure { "·" } else { " " };

    let line = Line::from(vec![
        Span::styled(format!("{infra_marker}"), Style::default().fg(Color::DarkGray)),
        Span::styled(spinner, Style::default().fg(Color::Green)),
        Span::styled(
            format!("[{state_label}]"),
            state_style.add_modifier(Modifier::BOLD),
        ),
        Span::raw("  "),
        Span::styled(
            format!("{:<16}", def.name),
            if selected {
                Style::default().fg(Color::White).add_modifier(Modifier::BOLD)
            } else {
                Style::default()
            },
        ),
        Span::raw("  "),
        Span::styled("s", Style::default().fg(Color::Yellow)),
        Span::raw(":start/stop  "),
        Span::styled("r", Style::default().fg(Color::Yellow)),
        Span::raw(":restart  "),
        Span::styled("l", Style::default().fg(Color::Yellow)),
        Span::raw(":logs"),
    ]);

    let p = Paragraph::new(line).style(row_style);
    f.render_widget(p, area);
}

fn draw_logs(f: &mut Frame, app: &App, comp_idx: usize) {
    let area = f.size();
    let comp = &app.components[comp_idx];
    let def = &COMPONENT_DEFS[comp_idx];

    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Length(3), Constraint::Min(0), Constraint::Length(1)])
        .split(area);

    let title = format!(" Logs: {} ", def.name);
    let header = Paragraph::new(title)
        .block(Block::default().borders(Borders::ALL))
        .style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD));
    f.render_widget(header, chunks[0]);

    let log_area = chunks[1];
    let log_height = log_area.height as usize;

    let lines: Vec<Line> = {
        let logs = comp.logs.lock().unwrap();
        let total = logs.len();
        let scroll = app.log_scroll.min(if total > log_height { total - log_height } else { 0 });
        logs.iter()
            .skip(scroll)
            .take(log_height)
            .map(|s| Line::from(s.clone()))
            .collect()
    };

    let log_p = Paragraph::new(lines)
        .block(Block::default().borders(Borders::ALL))
        .style(Style::default().fg(Color::White));
    f.render_widget(log_p, log_area);

    let footer = Paragraph::new(" ↑↓/PgUp/PgDn: scroll  Esc/q: back  G: jump to end")
        .style(Style::default().fg(Color::DarkGray));
    f.render_widget(footer, chunks[2]);
}

// ─── Input handling ───────────────────────────────────────────────────────────

async fn handle_config_key(app: &mut App, key: KeyCode, modifiers: KeyModifiers) -> bool {
    match (&app.config_focus.clone(), key) {
        // Navigation
        (_, KeyCode::Tab) => {
            if modifiers.contains(KeyModifiers::SHIFT) {
                app.config_focus_prev();
            } else {
                app.config_focus_next();
            }
        }
        (_, KeyCode::Down) => app.config_focus_next(),
        (_, KeyCode::Up) => app.config_focus_prev(),

        // Zenoh toggle
        (ConfigFocus::Zenoh, KeyCode::Char(' ') | KeyCode::Enter) => {
            app.params.zenoh = !app.params.zenoh;
            let zenoh_idx = COMPONENT_DEFS.iter().position(|d| d.key == "zenoh").unwrap();
            app.components[zenoh_idx].enabled = app.params.zenoh;
        }

        // Sim toggle
        (ConfigFocus::Sim, KeyCode::Char(' ') | KeyCode::Enter) => {
            app.params.sim = !app.params.sim;
            let sim = app.params.sim;
            for (i, def) in COMPONENT_DEFS.iter().enumerate() {
                if def.key == "simulator" {
                    app.components[i].enabled = sim;
                } else if def.hardware_only {
                    // Auto-disable HW-only components in sim, restore default otherwise
                    app.components[i].enabled = !sim && def.default_enabled;
                }
            }
            // Reset config focus if it now points at a hidden component
            app.config_focus = ConfigFocus::Zenoh;
        }

        // Fieldname text input
        (ConfigFocus::Fieldname, KeyCode::Char(c)) => {
            app.config_fieldname_input.push(c);
            app.params.fieldname = app.config_fieldname_input.clone();
        }
        (ConfigFocus::Fieldname, KeyCode::Backspace) => {
            app.config_fieldname_input.pop();
            app.params.fieldname = app.config_fieldname_input.clone();
        }

        // DSD file text input
        (ConfigFocus::DsdFile, KeyCode::Char(c)) => {
            app.config_dsd_input.push(c);
            app.params.dsd_file = app.config_dsd_input.clone();
        }
        (ConfigFocus::DsdFile, KeyCode::Backspace) => {
            app.config_dsd_input.pop();
            app.params.dsd_file = app.config_dsd_input.clone();
        }

        // Component checkbox toggle
        (ConfigFocus::Component(fi), KeyCode::Char(' ') | KeyCode::Enter) => {
            let fi = *fi;
            let toggleables = app.toggleable_indices();
            if fi < toggleables.len() {
                let comp_idx = toggleables[fi];
                app.components[comp_idx].enabled = !app.components[comp_idx].enabled;
            }
        }

        // Start button
        (ConfigFocus::Start, KeyCode::Enter | KeyCode::Char(' ')) => {
            // Transition to runtime
            app.params.fieldname = app.config_fieldname_input.clone();
            app.params.dsd_file = app.config_dsd_input.clone();
            app.screen = Screen::Runtime;
            app.start_all_enabled().await;
        }

        // Quit
        (_, KeyCode::Char('q') | KeyCode::Char('Q')) => {
            return true; // signal quit
        }
        (_, KeyCode::Esc) => {
            return true;
        }

        _ => {}
    }
    false
}

async fn handle_runtime_key(app: &mut App, key: KeyCode, _modifiers: KeyModifiers) -> bool {
    let visible = app.runtime_visible_indices();
    let sel = app.runtime_selected.min(visible.len().saturating_sub(1));
    let comp_idx = visible.get(sel).copied();

    match key {
        KeyCode::Down | KeyCode::Char('j') => {
            app.runtime_selected = (sel + 1).min(visible.len().saturating_sub(1));
        }
        KeyCode::Up | KeyCode::Char('k') => {
            app.runtime_selected = sel.saturating_sub(1);
        }
        KeyCode::Char('s') => {
            if let Some(idx) = comp_idx {
                match app.components[idx].state {
                    ProcState::Running | ProcState::Stopping => {
                        app.stop_component(idx).await;
                    }
                    ProcState::Idle | ProcState::Stopped | ProcState::Crashed => {
                        app.components[idx].state = ProcState::Idle;
                        app.start_component(idx).await;
                    }
                }
            }
        }
        KeyCode::Char('r') => {
            if let Some(idx) = comp_idx {
                app.stop_component(idx).await;
                // Brief delay then restart — we set idle so it auto-starts
                // Actually just start immediately; stop sends SIGTERM async
                app.components[idx].state = ProcState::Idle;
                app.start_component(idx).await;
            }
        }
        KeyCode::Char('l') => {
            if let Some(idx) = comp_idx {
                app.log_scroll = usize::MAX; // jump to end
                {
                    let logs = app.components[idx].logs.lock().unwrap();
                    let len = logs.len();
                    drop(logs);
                    app.log_scroll = len.saturating_sub(1);
                }
                app.screen = Screen::Logs(idx);
            }
        }
        KeyCode::Char('a') => {
            app.start_all_enabled().await;
        }
        KeyCode::Char('x') => {
            app.stop_all().await;
        }
        KeyCode::Esc => {
            // Back to config (ask for quit confirm instead)
            app.screen = Screen::Config;
        }
        KeyCode::Char('q') | KeyCode::Char('Q') => {
            return true;
        }
        _ => {}
    }
    false
}

async fn handle_logs_key(app: &mut App, key: KeyCode, _modifiers: KeyModifiers, comp_idx: usize) -> bool {
    let log_len = app.components[comp_idx].logs.lock().unwrap().len();
    match key {
        KeyCode::Down | KeyCode::Char('j') => {
            app.log_scroll = (app.log_scroll + 1).min(log_len.saturating_sub(1));
        }
        KeyCode::Up | KeyCode::Char('k') => {
            app.log_scroll = app.log_scroll.saturating_sub(1);
        }
        KeyCode::PageDown => {
            app.log_scroll = (app.log_scroll + 20).min(log_len.saturating_sub(1));
        }
        KeyCode::PageUp => {
            app.log_scroll = app.log_scroll.saturating_sub(20);
        }
        KeyCode::Char('G') => {
            app.log_scroll = log_len.saturating_sub(1);
        }
        KeyCode::Char('g') => {
            app.log_scroll = 0;
        }
        KeyCode::Esc | KeyCode::Char('q') | KeyCode::Char('l') => {
            app.screen = Screen::Runtime;
        }
        _ => {}
    }
    false
}

// ─── Terminal cleanup helpers ─────────────────────────────────────────────────

fn setup_panic_hook() {
    let original = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        // Best-effort cleanup — ignore errors
        let _ = disable_raw_mode();
        let _ = execute!(
            std::io::stdout(),
            LeaveAlternateScreen,
            DisableMouseCapture
        );
        original(info);
    }));
}

fn cleanup_terminal() {
    let _ = disable_raw_mode();
    let _ = execute!(
        std::io::stdout(),
        LeaveAlternateScreen,
        DisableMouseCapture
    );
}

// ─── Main ─────────────────────────────────────────────────────────────────────

#[tokio::main]
async fn main() -> Result<()> {
    setup_panic_hook();

    // Setup terminal
    enable_raw_mode()?;
    let mut stdout = std::io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let mut app = App::new();
    let mut event_stream = EventStream::new();
    let mut tick_interval = tokio::time::interval(Duration::from_millis(100));

    let result = run_loop(&mut terminal, &mut app, &mut event_stream, &mut tick_interval).await;

    // Ensure all children are killed
    app.stop_all().await;
    // Give them a moment to receive SIGTERM
    tokio::time::sleep(Duration::from_millis(300)).await;
    // SIGKILL anything still alive (whole process group)
    for comp in &app.components {
        for &pid in &comp.pids {
            unsafe {
                libc::kill(-(pid as libc::pid_t), libc::SIGKILL);
            }
        }
    }

    cleanup_terminal();
    result
}

async fn run_loop(
    terminal: &mut Terminal<CrosstermBackend<std::io::Stdout>>,
    app: &mut App,
    event_stream: &mut EventStream,
    tick_interval: &mut tokio::time::Interval,
) -> Result<()> {
    loop {
        terminal.draw(|f| draw(f, app))?;

        tokio::select! {
            _ = tick_interval.tick() => {
                app.tick_count += 1;
                app.drain_messages();
                // Auto-scroll logs to bottom if on log screen
                if let Screen::Logs(idx) = &app.screen {
                    let len = app.components[*idx].logs.lock().unwrap().len();
                    // Only auto-scroll if near the bottom
                    if app.log_scroll + 5 >= len.saturating_sub(1) {
                        app.log_scroll = len.saturating_sub(1);
                    }
                }
            }

            maybe_event = event_stream.next() => {
                let Some(Ok(event)) = maybe_event else { break; };

                if let Event::Key(key_event) = event {
                    let quit = match app.screen.clone() {
                        Screen::Config => {
                            handle_config_key(app, key_event.code, key_event.modifiers).await
                        }
                        Screen::Runtime => {
                            handle_runtime_key(app, key_event.code, key_event.modifiers).await
                        }
                        Screen::Logs(idx) => {
                            handle_logs_key(app, key_event.code, key_event.modifiers, idx).await
                        }
                    };
                    if quit {
                        return Ok(());
                    }
                }
            }

            _ = tokio::signal::ctrl_c() => {
                return Ok(());
            }
        }
    }
    Ok(())
}
