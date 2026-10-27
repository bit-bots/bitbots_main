use ratatui::{
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{
        Block, Borders, Clear, Paragraph, Scrollbar, ScrollbarOrientation, ScrollbarState, Wrap,
    },
    Frame,
};

use bitbots_orchestrate_api::api::{ParamType, ProcState, RosState, Value};

use crate::app::{App, ConfigFocus, ConfirmAction, Screen};

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
        ProcState::Waiting => Style::default().fg(Color::Magenta),
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
    if app.status.is_none() {
        draw_waiting(f, app);
        return;
    }
    match &app.screen {
        Screen::Config => draw_config(f, app),
        Screen::Runtime => draw_runtime(f, app),
        Screen::Logs(idx) => draw_logs(f, app, *idx),
        Screen::AllLogs => draw_all_logs(f, app),
    }
    if app.confirm_action.is_some() {
        draw_confirm_popup(f, app);
    }
    draw_error_line(f, app);
}

/// Shown until the orchestrator has been reached once.
fn draw_waiting(f: &mut Frame, app: &App) {
    let area = f.area();
    let reason = app
        .orchestrator_failure
        .as_deref()
        .or(app.connection_error.as_deref())
        .unwrap_or("connecting…");
    let text = format!(
        "{BANNER}\n\nWaiting for bitbots_orchestrate at {}\n\n{reason}\n\n\
         Start it with `pixi run orchestrate`, or run teamplayer without --attach\n\nq: quit",
        app.client.socket().display()
    );
    f.render_widget(
        Paragraph::new(text)
            .style(Style::default().fg(Color::Cyan))
            .alignment(Alignment::Center)
            .wrap(Wrap { trim: false }),
        area,
    );
}

/// Replaces the bottom hint line while the orchestrator is unreachable or refused an action.
fn draw_error_line(f: &mut Frame, app: &App) {
    let mut problems: Vec<String> = app
        .status
        .as_ref()
        .map(|s| s.problems.clone())
        .unwrap_or_default();
    if let Some(e) = app
        .status
        .as_ref()
        .and_then(|s| s.blackboard.as_ref())
        .and_then(|bb| bb.error.as_ref())
    {
        problems.push(format!("blackboard: {e}"));
    }
    let problems = problems.join("; ");
    let (msg, bg) = match (&app.connection_error, &app.action_error) {
        _ if app.orchestrator_failure.is_some() => (
            format!(
                " {}",
                app.orchestrator_failure.as_deref().unwrap_or_default()
            ),
            Color::Red,
        ),
        (Some(e), _) => (format!(" orchestrator unreachable: {e}"), Color::Red),
        (None, Some(e)) => (format!(" {e}"), Color::Red),
        (None, None) if !problems.is_empty() => (format!(" {problems}"), Color::Yellow),
        (None, None) => return,
    };
    let area = f.area();
    if area.height == 0 {
        return;
    }
    let line = Rect::new(area.x, area.y + area.height - 1, area.width, 1);
    f.render_widget(Clear, line);
    f.render_widget(
        Paragraph::new(msg).style(Style::default().fg(Color::Black).bg(bg)),
        line,
    );
}

// ─── Config screen ────────────────────────────────────────────────────────────

pub fn draw_config(f: &mut Frame, app: &App) {
    let area = f.area();
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

    let hint = match app.focused_description() {
        Some(description) => format!(" {description}"),
        None => " Tab/↑↓/←→: navigate  Space/Enter: toggle/activate  q: quit".to_string(),
    };
    f.render_widget(
        Paragraph::new(hint).style(Style::default().fg(Color::DarkGray)),
        chunks[4],
    );
}

fn draw_config_flags(f: &mut Frame, app: &App, area: Rect) {
    let params = app.parameters();
    let constraints: Vec<Constraint> = [Constraint::Length(20)]
        .into_iter()
        .chain(params.iter().map(|p| match p.param_type {
            ParamType::Bool => Constraint::Length(p.name.chars().count() as u16 + 8),
            ParamType::String => Constraint::Min(16),
        }))
        .collect();
    let chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints(constraints)
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

    let profile_focused = app.config_focus == ConfigFocus::Profile;
    let profile = app
        .status
        .as_ref()
        .and_then(|s| s.profile.as_deref())
        .unwrap_or("defaults");
    f.render_widget(
        Paragraph::new(profile)
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title("Profile")
                    .border_style(focus_border(profile_focused)),
            )
            .style(focus_text(profile_focused)),
        chunks[0],
    );

    for (i, param) in params.iter().enumerate() {
        let focused = app.config_focus == ConfigFocus::Param(i);
        let widget = match &param.value {
            Value::Bool(b) => {
                let check = if *b { "✓" } else { " " };
                Paragraph::new(format!("[{check}] {}", param.name))
                    .block(
                        Block::default()
                            .borders(Borders::ALL)
                            .border_style(focus_border(focused)),
                    )
                    .style(focus_text(focused))
            }
            Value::String(value) if param.choices.is_some() => {
                Paragraph::new(format!("‹ {value} ›"))
                    .block(
                        Block::default()
                            .borders(Borders::ALL)
                            .title(param.name.as_str())
                            .border_style(focus_border(focused)),
                    )
                    .style(focus_text(focused))
            }
            Value::String(_) => {
                let input = app.param_inputs.get(&param.name).map(String::as_str);
                Paragraph::new(input.unwrap_or_default().to_string()).block(
                    Block::default()
                        .borders(Borders::ALL)
                        .title(param.name.as_str())
                        .border_style(focus_border(focused)),
                )
            }
        };
        f.render_widget(widget, chunks[i + 1]);
    }
}

fn draw_config_components(f: &mut Frame, app: &App, area: Rect) {
    let toggleables = app.toggleable_indices();
    let col_chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(area);

    let left_count = toggleables.len().div_ceil(2);
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
        let comp = &app.components()[comp_idx];

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
                Paragraph::new(format!("[{check}] {}", comp.name)).style(style),
                chunks[local_i],
            );
        }
    }
}

fn draw_start_button(f: &mut Frame, app: &App, area: Rect) {
    let focused = app.config_focus == ConfigFocus::Start;
    let has_running = app.components().iter().any(|c| c.state.is_alive());
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
    let area = f.area();
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3),
            Constraint::Min(0),
            Constraint::Length(1),
        ])
        .split(area);

    draw_runtime_header(f, app, chunks[0]);

    if app.show_log_panel {
        let cols = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([Constraint::Percentage(35), Constraint::Percentage(65)])
            .split(chunks[1]);
        draw_runtime_list(f, app, cols[0]);
        draw_joint_logs(f, app, cols[1]);
    } else {
        draw_runtime_list(f, app, chunks[1]);
    }

    let hint = if app.show_log_panel {
        " ↑↓/jk: select  l: comp logs  →: all logs  ←: hide logs  s: stop/start  r: restart  a: start all  x: stop all  Esc: config  q: quit"
    } else {
        " ↑↓/jk: select  l: comp logs  →: show logs  s: stop/start  r: restart  a: start all  x: stop all  Esc: config  q: quit"
    };
    f.render_widget(
        Paragraph::new(hint).style(Style::default().fg(Color::DarkGray)),
        chunks[2],
    );
}

fn draw_runtime_header(f: &mut Frame, app: &App, area: Rect) {
    let running = app
        .components()
        .iter()
        .filter(|c| c.state == ProcState::Running)
        .count();
    let total_enabled = app.components().iter().filter(|c| c.enabled).count();
    let profile_badge = match app.status.as_ref().and_then(|s| s.profile.as_deref()) {
        Some(profile) => format!("  [{profile}]"),
        None => String::new(),
    };
    let blackboard_badge = match app.status.as_ref().and_then(|s| s.blackboard.as_ref()) {
        Some(bb) if bb.error.is_some() => "  [blackboard: ERROR]".to_string(),
        Some(bb) => match &bb.ros {
            RosState::Running => "  [blackboard]".to_string(),
            RosState::WaitingFor(c) if c.is_empty() => "  [blackboard: waiting]".to_string(),
            RosState::WaitingFor(c) => format!("  [blackboard: waiting for {}]", c.join(", ")),
            RosState::Starting => "  [blackboard: starting]".to_string(),
            RosState::Failed(_) => "  [blackboard: FAILED]".to_string(),
        },
        None => String::new(),
    };
    f.render_widget(
        Paragraph::new(format!(
            " Bit-Bots Teamplayer  [{running}/{total_enabled} running]{profile_badge}{blackboard_badge}"
        ))
        .block(Block::default().borders(Borders::ALL))
        .style(
            Style::default()
                .fg(Color::Cyan)
                .add_modifier(Modifier::BOLD),
        ),
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
        let comp = &app.components()[comp_idx];

        let row_style = if selected {
            Style::default().bg(Color::DarkGray)
        } else {
            Style::default()
        };
        let spinner = match comp.state {
            ProcState::Running => format!("{} ", spinner_char(app.tick_count)),
            ProcState::Stopping | ProcState::Waiting => "… ".to_string(),
            ProcState::Restarting => "↺ ".to_string(),
            _ => "  ".to_string(),
        };

        let line = Line::from(vec![
            Span::styled(
                if comp.locked {
                    "·"
                } else if comp.fallback_for.is_some() {
                    "↪"
                } else {
                    " "
                },
                Style::default().fg(Color::DarkGray),
            ),
            Span::styled(spinner, Style::default().fg(Color::Green)),
            Span::styled(
                format!("[{}]", comp.state.label()),
                proc_state_style(&comp.state).add_modifier(Modifier::BOLD),
            ),
            Span::raw("  "),
            Span::styled(
                format!("{:<18}", comp.name),
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

fn tag_color(tag: &str) -> Color {
    const PALETTE: &[Color] = &[
        Color::Cyan,
        Color::Green,
        Color::Magenta,
        Color::Yellow,
        Color::LightBlue,
        Color::LightGreen,
        Color::LightMagenta,
        Color::LightRed,
    ];
    let h = tag.bytes().fold(0usize, |a, b| a.wrapping_add(b as usize));
    PALETTE[h % PALETTE.len()]
}

/// Convert a raw log string (which may contain ANSI escape codes) into an owned ratatui Line.
fn ansi_to_line(s: &str) -> Line<'static> {
    use ansi_to_tui::IntoText;
    match s.into_text() {
        Ok(text) => Line::from(
            text.lines
                .into_iter()
                .flat_map(|l| l.spans)
                .map(|s| Span::styled(s.content.into_owned(), s.style))
                .collect::<Vec<_>>(),
        ),
        Err(_) => Line::from(s.to_owned()),
    }
}

fn draw_joint_logs(f: &mut Frame, app: &App, area: Rect) {
    let inner_h = area.height.saturating_sub(2) as usize;
    let inner_w = area.width.saturating_sub(2) as usize;

    let lines: Vec<Line<'static>> = {
        let logs = &app.all_logs.lines;
        // Grab more entries than the panel height — wrapping will consume extra rows.
        let skip = logs.len().saturating_sub(inner_h * 3);
        logs.iter()
            .skip(skip)
            .map(|entry| {
                if let Some(close) = entry.find("] ") {
                    let tag = &entry[..close + 1]; // "[CompName]"
                    let rest = &entry[close + 2..];
                    let mut spans: Vec<Span<'static>> = vec![
                        Span::styled(
                            tag.to_owned(),
                            Style::default()
                                .fg(tag_color(tag))
                                .add_modifier(Modifier::BOLD),
                        ),
                        Span::raw(" "),
                    ];
                    spans.extend(ansi_to_line(rest).spans);
                    Line::from(spans)
                } else {
                    ansi_to_line(entry)
                }
            })
            .collect()
    };

    // Estimate total visual rows so we can scroll to pin the view to the bottom.
    let total_rows: usize = lines
        .iter()
        .map(|l| {
            let chars: usize = l.spans.iter().map(|s| s.content.chars().count()).sum();
            if inner_w > 0 {
                chars.div_ceil(inner_w).max(1)
            } else {
                1
            }
        })
        .sum();
    let scroll = total_rows.saturating_sub(inner_h) as u16;

    f.render_widget(
        Paragraph::new(lines)
            .block(Block::default().borders(Borders::ALL).title(" All Logs "))
            .wrap(Wrap { trim: false })
            .scroll((scroll, 0)),
        area,
    );
}

// ─── Logs screen ──────────────────────────────────────────────────────────────

pub fn draw_logs(f: &mut Frame, app: &App, comp_idx: usize) {
    let area = f.area();
    let name = app
        .components()
        .get(comp_idx)
        .map(|c| c.name.as_str())
        .unwrap_or("?");

    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3),
            Constraint::Min(0),
            Constraint::Length(1),
        ])
        .split(area);

    f.render_widget(
        Paragraph::new(format!(" Logs: {name} "))
            .block(Block::default().borders(Borders::ALL))
            .style(
                Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD),
            ),
        chunks[0],
    );

    let log_height = chunks[1].height.saturating_sub(2) as usize; // inner height (minus borders)
    app.log_viewport_height.set(log_height);
    let (lines, total, scroll) = {
        let logs = &app.component_logs.lines;
        let total = logs.len();
        let max_scroll = total.saturating_sub(log_height);
        let scroll = app.log_scroll.min(max_scroll);
        let lines: Vec<Line<'static>> = logs
            .iter()
            .skip(scroll)
            .take(log_height * 2)
            .map(|s| ansi_to_line(s))
            .collect();
        (lines, total, scroll)
    };

    let log_block = Block::default().borders(Borders::ALL);
    let log_area = chunks[1];
    f.render_widget(
        Paragraph::new(lines)
            .block(log_block)
            .style(Style::default().fg(Color::White))
            .wrap(Wrap { trim: false }),
        log_area,
    );

    let mut scrollbar_state =
        ScrollbarState::new(total.saturating_sub(log_height)).position(scroll);
    f.render_stateful_widget(
        Scrollbar::new(ScrollbarOrientation::VerticalRight),
        log_area,
        &mut scrollbar_state,
    );

    let hint = if app.log_follow {
        " ↑↓/jk/PgUp/PgDn: scroll  g: top  G/f: follow  Esc/q/l: back".to_string()
    } else {
        " ↑↓/jk/PgUp/PgDn: scroll  g: top  G/f: follow  Esc/q/l: back  [SCROLLED — f/G to re-follow]".to_string()
    };
    let hint_style = if app.log_follow {
        Style::default().fg(Color::DarkGray)
    } else {
        Style::default().fg(Color::Yellow)
    };
    f.render_widget(Paragraph::new(hint).style(hint_style), chunks[2]);
}

// ─── All-logs screen ──────────────────────────────────────────────────────────

pub fn draw_all_logs(f: &mut Frame, app: &App) {
    let area = f.area();
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Min(0), Constraint::Length(1)])
        .split(area);

    let log_height = chunks[0].height.saturating_sub(2) as usize;
    app.log_viewport_height.set(log_height);
    let (lines, total, scroll) = {
        let logs = &app.all_logs.lines;
        let total = logs.len();
        let max_scroll = total.saturating_sub(log_height);
        let scroll = app.log_scroll.min(max_scroll);
        let lines: Vec<Line<'static>> = logs
            .iter()
            .skip(scroll)
            .take(log_height * 2)
            .map(|entry| {
                if let Some(close) = entry.find("] ") {
                    let tag = &entry[..close + 1];
                    let rest = &entry[close + 2..];
                    let mut spans: Vec<Span<'static>> = vec![
                        Span::styled(
                            tag.to_owned(),
                            Style::default()
                                .fg(tag_color(tag))
                                .add_modifier(Modifier::BOLD),
                        ),
                        Span::raw(" "),
                    ];
                    spans.extend(ansi_to_line(rest).spans);
                    Line::from(spans)
                } else {
                    ansi_to_line(entry)
                }
            })
            .collect();
        (lines, total, scroll)
    };

    let log_area = chunks[0];
    f.render_widget(
        Paragraph::new(lines)
            .block(Block::default().borders(Borders::ALL).title(" All Logs "))
            .style(Style::default().fg(Color::White))
            .wrap(Wrap { trim: false }),
        log_area,
    );

    let mut scrollbar_state =
        ScrollbarState::new(total.saturating_sub(log_height)).position(scroll);
    f.render_stateful_widget(
        Scrollbar::new(ScrollbarOrientation::VerticalRight),
        log_area,
        &mut scrollbar_state,
    );

    let hint = if app.log_follow {
        " ↑↓/jk/PgUp/PgDn: scroll  g: top  G/f: follow  Esc/q/l/←: back"
    } else {
        " ↑↓/jk/PgUp/PgDn: scroll  g: top  G/f: follow  Esc/q/l/←: back  [SCROLLED — f/G to re-follow]"
    };
    let hint_style = if app.log_follow {
        Style::default().fg(Color::DarkGray)
    } else {
        Style::default().fg(Color::Yellow)
    };
    f.render_widget(Paragraph::new(hint).style(hint_style), chunks[1]);
}

// ─── Confirmation popup ───────────────────────────────────────────────────────

pub fn draw_confirm_popup(f: &mut Frame, app: &App) {
    let (title, msg) = match &app.confirm_action {
        Some(ConfirmAction::StopAll) => (
            " Stop All ",
            "Stop all running components?\n\nY / Enter — confirm    N / Esc — cancel",
        ),
        Some(ConfirmAction::Quit) => (
            " Quit ",
            "This teamplayer started the orchestrator.\n\n\
             Y / Enter — stop all components and quit\n\
             D — quit, keep components running\n\
             N / Esc — cancel",
        ),
        None => return,
    };

    let area = f.area();
    let popup_w = 52u16.min(area.width.saturating_sub(4));
    let popup_h = msg.lines().count() as u16 + 2; // + borders
    let x = area.x + (area.width.saturating_sub(popup_w)) / 2;
    let y = area.y + (area.height.saturating_sub(popup_h)) / 2;
    let popup_area = Rect::new(x, y, popup_w, popup_h);

    f.render_widget(Clear, popup_area);

    let content = msg;
    f.render_widget(
        Paragraph::new(content)
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title(title)
                    .border_style(Style::default().fg(Color::Yellow)),
            )
            .style(
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD),
            )
            .alignment(Alignment::Center),
        popup_area,
    );
}
