use crossterm::event::{KeyCode, KeyModifiers};

use crate::app::{App, ConfigFocus, ConfirmAction, ProcState, Screen};
use crate::components::COMPONENT_DEFS;

pub async fn handle_config_key(app: &mut App, key: KeyCode, modifiers: KeyModifiers) -> bool {
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
        (_, KeyCode::Right) => app.config_focus_right(),
        (_, KeyCode::Left) => app.config_focus_left(),

        // Zenoh toggle
        (ConfigFocus::Zenoh, KeyCode::Char(' ') | KeyCode::Enter) => {
            app.params.zenoh = !app.params.zenoh;
            if let Some(idx) = COMPONENT_DEFS.iter().position(|d| d.key == "zenoh") {
                app.components[idx].enabled = app.params.zenoh;
            }
        }

        // Sim toggle
        (ConfigFocus::Sim, KeyCode::Char(' ') | KeyCode::Enter) => {
            app.params.sim = !app.params.sim;
            app.apply_sim_toggle();
        }

        // Fieldname text input
        (ConfigFocus::Fieldname, KeyCode::Char(c)) => {
            if !modifiers.contains(KeyModifiers::CONTROL) {
                app.config_fieldname_input.push(c);
                app.params.fieldname = app.config_fieldname_input.clone();
            }
        }
        (ConfigFocus::Fieldname, KeyCode::Backspace) => {
            app.config_fieldname_input.pop();
            app.params.fieldname = app.config_fieldname_input.clone();
        }

        // DSD file text input
        (ConfigFocus::DsdFile, KeyCode::Char(c)) => {
            if !modifiers.contains(KeyModifiers::CONTROL) {
                app.config_dsd_input.push(c);
                app.params.dsd_file = app.config_dsd_input.clone();
            }
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

        // Start / Continue button
        (ConfigFocus::Start, KeyCode::Enter | KeyCode::Char(' ')) => {
            app.params.fieldname = app.config_fieldname_input.clone();
            app.params.dsd_file = app.config_dsd_input.clone();
            app.screen = Screen::Runtime;
            app.start_all_enabled().await;
        }

        // Quit
        (_, KeyCode::Char('q') | KeyCode::Char('Q') | KeyCode::Esc) => {
            app.confirm_action = Some(ConfirmAction::Quit);
        }

        _ => {}
    }
    false
}

pub async fn handle_runtime_key(app: &mut App, key: KeyCode, _modifiers: KeyModifiers) -> bool {
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
                    ProcState::Running | ProcState::Stopping | ProcState::Restarting => {
                        app.stop_component(idx).await;
                    }
                    _ => {
                        app.components[idx].state = ProcState::Idle;
                        app.start_component(idx).await;
                    }
                }
            }
        }

        KeyCode::Char('r') => {
            if let Some(idx) = comp_idx {
                match app.components[idx].state {
                    ProcState::Running => app.restart_component(idx),
                    ProcState::Stopping | ProcState::Restarting => {}
                    _ => app.start_component(idx).await,
                }
            }
        }

        KeyCode::Char('l') => {
            if let Some(idx) = comp_idx {
                app.log_scroll = app.components[idx]
                    .logs
                    .lock()
                    .unwrap()
                    .len()
                    .saturating_sub(1);
                app.log_follow = true;
                app.screen = Screen::Logs(idx);
            }
        }

        // Right: show sidebar if hidden, open AllLogs if sidebar already visible
        KeyCode::Right => {
            if app.show_log_panel {
                app.log_scroll = app.all_logs.lock().unwrap().len().saturating_sub(1);
                app.log_follow = true;
                app.screen = Screen::AllLogs;
            } else {
                app.show_log_panel = true;
            }
        }

        // Left: hide sidebar
        KeyCode::Left => {
            app.show_log_panel = false;
        }

        KeyCode::Char('a') => app.start_all_enabled().await,
        KeyCode::Char('x') => app.confirm_action = Some(ConfirmAction::StopAll),
        KeyCode::Esc => app.screen = Screen::Config,
        KeyCode::Char('q') | KeyCode::Char('Q') => {
            app.confirm_action = Some(ConfirmAction::Quit);
        }
        _ => {}
    }
    false
}

pub async fn handle_logs_key(
    app: &mut App,
    key: KeyCode,
    _modifiers: KeyModifiers,
    comp_idx: usize,
) -> bool {
    let log_len = app.components[comp_idx].logs.lock().unwrap().len();
    scroll_keys(app, key, log_len);
    match key {
        KeyCode::Esc | KeyCode::Char('q') | KeyCode::Char('l') => {
            app.screen = Screen::Runtime;
        }
        KeyCode::Char('Q') => app.confirm_action = Some(ConfirmAction::Quit),
        _ => {}
    }
    false
}

pub fn handle_all_logs_key(app: &mut App, key: KeyCode) -> bool {
    let log_len = app.all_logs.lock().unwrap().len();
    scroll_keys(app, key, log_len);
    match key {
        KeyCode::Esc | KeyCode::Char('q') | KeyCode::Char('l') | KeyCode::Left => {
            app.screen = Screen::Runtime;
        }
        KeyCode::Char('Q') => app.confirm_action = Some(ConfirmAction::Quit),
        _ => {}
    }
    false
}

/// Shared scroll key handling for both log views.
fn scroll_keys(app: &mut App, key: KeyCode, log_len: usize) {
    let vh = app.log_viewport_height.get();
    let max_scroll = log_len.saturating_sub(vh.max(1));
    match key {
        KeyCode::Down | KeyCode::Char('j') => {
            let next = (app.log_scroll + 1).min(max_scroll);
            app.log_scroll = next;
            app.log_follow = next >= max_scroll;
        }
        KeyCode::Up | KeyCode::Char('k') => {
            app.log_scroll = app.log_scroll.saturating_sub(1);
            app.log_follow = false;
        }
        KeyCode::PageDown => {
            let next = (app.log_scroll + 20).min(max_scroll);
            app.log_scroll = next;
            app.log_follow = next >= max_scroll;
        }
        KeyCode::PageUp => {
            app.log_scroll = app.log_scroll.saturating_sub(20);
            app.log_follow = false;
        }
        KeyCode::Char('G') | KeyCode::Char('f') => {
            app.log_scroll = max_scroll;
            app.log_follow = true;
        }
        KeyCode::Char('g') => {
            app.log_scroll = 0;
            app.log_follow = false;
        }
        _ => {}
    }
}

pub async fn handle_confirm_key(app: &mut App, key: KeyCode) -> bool {
    match key {
        KeyCode::Char('y') | KeyCode::Char('Y') | KeyCode::Enter => {
            match app.confirm_action.take() {
                Some(ConfirmAction::Quit) => return true,
                Some(ConfirmAction::StopAll) => app.stop_all().await,
                None => {}
            }
        }
        _ => {
            app.confirm_action = None;
        }
    }
    false
}
