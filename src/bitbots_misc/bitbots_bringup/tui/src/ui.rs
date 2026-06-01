use ratatui::{
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Paragraph},
    Frame,
};

use crate::app::{App, ConfigFocus, ProcState, Screen};
use crate::components::COMPONENT_DEFS;

// ─── Banner ───────────────────────────────────────────────────────────────────

const BANNER: &str = r#"
  ██████╗ ██╗████████╗    ██████╗  ██████╗ ████████╗███████╗
  ██╔══██╗██║╚══██╔══╝    ██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝
  ██████╔╝██║   ██║       ██████╔╝██║   ██║   ██║   ███████╗
  ██╔══██╗██║   ██║       ██╔══██╗██║   ██║   ██║   ╚════██║
  ██████╔╝██║   ██║       ██████╔╝╚██████╔╝   ██║   ███████║
  ╚═════╝ ╚═╝   ╚═╝       ╚═════╝  ╚═════╝    ╚═╝   ╚══════╝
                     Teamplayer Launcher
"#;

// ─── Helpers ──────────────────────────────────────────────────────────────────

pub fn proc_state_style(state: &ProcState) -> Style {
    match state {
        ProcState::Idle => Style::default().fg(Color::DarkGray),
        ProcState::Running => Style::default().fg(Color::Green),
        ProcState::Stopping => Style::default().fg(Color::Yellow),
        ProcState::Restarting => Style::default().fg(Color::Cyan),
        ProcState::Stopped => Style::default().fg(Color::Blue),
        ProcState::Crashed => Style::default().fg(Color::Red),
    }
}

fn spinner_char(tick: u64) -> char {
    const FRAMES: &[char] = &['⠋', '⠙', '⠹', '⠸', '⠼', '⠴', '⠦', '⠧', '⠇', '⠏'];
    FRAMES[(tick as usize) % FRAMES.len()]
}

// ─── Top-level dispatch ───────────────────────────────────────────────────────

pub fn draw(f: &mut Frame, app: &App) {
    match &app.screen {
        Screen::Config => draw_config(f, app),
        Screen::Runtime => draw_runtime(f, app),
        Screen::Logs(idx) => draw_logs(f, app, *idx),
    }
}

// ─── Config screen ────────────────────────────────────────────────────────────

pub fn draw_config(f: &mut Frame, app: &App) {
    let area = f.size();
    let banner_lines = BANNER.lines().count() as u16;

    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(banner_lines),
            Constraint::Length(3),
            Constraint::Min(0),
            Constraint::Length(3),
            Constraint::Length(1),
        ])
        .split(area);

    f.render_widget(
        Paragraph::new(BANNER)
            .style(Style::default().fg(Color::Cyan))
            .alignment(Alignment::Center),
        chunks[0],
    );

    draw_config_flags(f, app, chunks[1]);
    draw_config_components(f, app, chunks[2]);
    draw_start_button(f, app, chunks[3]);

    f.render_widget(
        Paragraph::new(" Tab/↑↓: navigate  Space/Enter: toggle/activate  q: quit")
            .style(Style::default().fg(Color::DarkGray)),
        chunks[4],
    );
}

fn draw_config_flags(f: &mut Frame, app: &App, area: Rect) {
    let chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Length(18),
            Constraint::Length(18),
            Constraint::Min(20),
            Constraint::Min(20),
        ])
        .split(area);

    let focus_border = |focused: bool| {
        if focused {
            Style::default().fg(Color::Yellow)
        } else {
            Style::default()
        }
    };
    let focus_text = |focused: bool| {
        if focused {
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD)
        } else {
            Style::default().fg(Color::White)
        }
    };

    let zenoh_focused = app.config_focus == ConfigFocus::Zenoh;
    let sim_focused = app.config_focus == ConfigFocus::Sim;
    let field_focused = app.config_focus == ConfigFocus::Fieldname;
    let dsd_focused = app.config_focus == ConfigFocus::DsdFile;

    f.render_widget(
        Paragraph::new(if app.params.zenoh { "[✓] Zenoh" } else { "[ ] Zenoh" })
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .border_style(focus_border(zenoh_focused)),
            )
            .style(focus_text(zenoh_focused)),
        chunks[0],
    );

    f.render_widget(
        Paragraph::new(if app.params.sim {
            "[✓] Simulator"
        } else {
            "[ ] Simulator"
        })
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(focus_border(sim_focused)),
        )
        .style(focus_text(sim_focused)),
        chunks[1],
    );

    f.render_widget(
        Paragraph::new(app.config_fieldname_input.as_str()).block(
            Block::default()
                .borders(Borders::ALL)
                .title("Fieldname")
                .border_style(focus_border(field_focused)),
        ),
        chunks[2],
    );

    f.render_widget(
        Paragraph::new(app.config_dsd_input.as_str()).block(
            Block::default()
                .borders(Borders::ALL)
                .title("DSD File")
                .border_style(focus_border(dsd_focused)),
        ),
        chunks[3],
    );
}

fn draw_config_components(f: &mut Frame, app: &App, area: Rect) {
    let toggleables = app.toggleable_indices();
    let col_chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(area);

    let left_count = (toggleables.len() + 1) / 2;
    let (left, right) = toggleables.split_at(left_count);
    draw_component_column(f, app, col_chunks[0], left, 0);
    draw_component_column(f, app, col_chunks[1], right, left.len());
}

fn draw_component_column(
    f: &mut Frame,
    app: &App,
    area: Rect,
    comp_indices: &[usize],
    focus_offset: usize,
) {
    if comp_indices.is_empty() {
        return;
    }
    let rows: Vec<Constraint> = comp_indices.iter().map(|_| Constraint::Length(1)).collect();
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints(rows)
        .split(area);

    for (local_i, &comp_idx) in comp_indices.iter().enumerate() {
        let focused = app.config_focus == ConfigFocus::Component(focus_offset + local_i);
        let comp = &app.components[comp_idx];
        let def = &COMPONENT_DEFS[comp_idx];

        let check = if comp.enabled { "✓" } else { " " };
        let style = if focused {
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD)
        } else if comp.enabled {
            Style::default().fg(Color::White)
        } else {
            Style::default().fg(Color::DarkGray)
        };

        if local_i < chunks.len() {
            f.render_widget(
                Paragraph::new(format!("[{check}] {}", def.name)).style(style),
                chunks[local_i],
            );
        }
    }
}

fn draw_start_button(f: &mut Frame, app: &App, area: Rect) {
    let focused = app.config_focus == ConfigFocus::Start;
    let has_running = app.components.iter().any(|c| c.state == ProcState::Running);
    let label = if has_running {
        "  Continue →  "
    } else {
        "  Launch →  "
    };
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
            Style::default()
                .fg(Color::Green)
                .add_modifier(Modifier::BOLD),
        )
    };
    f.render_widget(
        Paragraph::new(label)
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .border_style(border_style),
            )
            .style(text_style)
            .alignment(Alignment::Center),
        area,
    );
}

// ─── Runtime screen ───────────────────────────────────────────────────────────

pub fn draw_runtime(f: &mut Frame, app: &App) {
    let area = f.size();
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Length(3), Constraint::Min(0), Constraint::Length(1)])
        .split(area);

    draw_runtime_header(f, app, chunks[0]);
    draw_runtime_list(f, app, chunks[1]);
    f.render_widget(
        Paragraph::new(
            " ↑↓/jk: select  s: start/stop  r: restart  l: logs  a: start all  x: stop all  Esc: config  q: quit",
        )
        .style(Style::default().fg(Color::DarkGray)),
        chunks[2],
    );
}

fn draw_runtime_header(f: &mut Frame, app: &App, area: Rect) {
    let running = app
        .components
        .iter()
        .filter(|c| c.state == ProcState::Running)
        .count();
    let total_enabled = app.components.iter().filter(|c| c.enabled).count();
    let sim_badge = if app.params.sim { "  [SIM]" } else { "" };
    f.render_widget(
        Paragraph::new(format!(
            " Bit-Bots Teamplayer  [{running}/{total_enabled} running]{sim_badge}"
        ))
        .block(Block::default().borders(Borders::ALL))
        .style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
        area,
    );
}

fn draw_runtime_list(f: &mut Frame, app: &App, area: Rect) {
    let visible = app.runtime_visible_indices();
    if visible.is_empty() {
        f.render_widget(
            Paragraph::new("No components enabled.").alignment(Alignment::Center),
            area,
        );
        return;
    }

    let rows: Vec<Constraint> = visible.iter().map(|_| Constraint::Length(1)).collect();
    let row_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints(rows)
        .split(area);

    for (vis_i, &comp_idx) in visible.iter().enumerate() {
        if vis_i >= row_chunks.len() {
            break;
        }
        let selected = app.runtime_selected == vis_i;
        let comp = &app.components[comp_idx];
        let def = &COMPONENT_DEFS[comp_idx];

        let row_style = if selected {
            Style::default().bg(Color::DarkGray)
        } else {
            Style::default()
        };
        let spinner = match comp.state {
            ProcState::Running => format!("{} ", spinner_char(app.tick_count)),
            ProcState::Stopping => "… ".to_string(),
            ProcState::Restarting => "↺ ".to_string(),
            _ => "  ".to_string(),
        };

        let line = Line::from(vec![
            Span::styled(
                if def.infrastructure { "·" } else { " " },
                Style::default().fg(Color::DarkGray),
            ),
            Span::styled(spinner, Style::default().fg(Color::Green)),
            Span::styled(
                format!("[{}]", comp.state.label()),
                proc_state_style(&comp.state).add_modifier(Modifier::BOLD),
            ),
            Span::raw("  "),
            Span::styled(
                format!("{:<18}", def.name),
                if selected {
                    Style::default()
                        .fg(Color::White)
                        .add_modifier(Modifier::BOLD)
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
        f.render_widget(Paragraph::new(line).style(row_style), row_chunks[vis_i]);
    }
}

// ─── Logs screen ──────────────────────────────────────────────────────────────

pub fn draw_logs(f: &mut Frame, app: &App, comp_idx: usize) {
    let area = f.size();
    let comp = &app.components[comp_idx];
    let name = COMPONENT_DEFS[comp_idx].name;

    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Length(3), Constraint::Min(0), Constraint::Length(1)])
        .split(area);

    f.render_widget(
        Paragraph::new(format!(" Logs: {name} "))
            .block(Block::default().borders(Borders::ALL))
            .style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
        chunks[0],
    );

    let log_height = chunks[1].height as usize;
    let lines: Vec<Line> = {
        let logs = comp.logs.lock().unwrap();
        let total = logs.len();
        let max_scroll = if total > log_height { total - log_height } else { 0 };
        let scroll = app.log_scroll.min(max_scroll);
        logs.iter()
            .skip(scroll)
            .take(log_height)
            .map(|s| Line::from(s.clone()))
            .collect()
    };

    f.render_widget(
        Paragraph::new(lines)
            .block(Block::default().borders(Borders::ALL))
            .style(Style::default().fg(Color::White)),
        chunks[1],
    );

    f.render_widget(
        Paragraph::new(" ↑↓/jk/PgUp/PgDn: scroll  g: top  G: bottom  Esc/q/l: back")
            .style(Style::default().fg(Color::DarkGray)),
        chunks[2],
    );
}
