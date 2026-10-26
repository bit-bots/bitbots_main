use bitbots_orchestrate::api::ConfigUpdate;
use crossterm::event::{KeyCode, KeyModifiers};

use crate::app::{App, ConfigFocus, ConfirmAction, Screen};

pub async fn handle_config_key(app: &mut App, key: KeyCode, modifiers: KeyModifiers) -> bool {
    match (&app.config_focus.clone(), key) {
        // Navigation
        (_, KeyCode::Tab) => {
            if modifiers.contains(KeyModifiers::SHIFT) {
                app.config_focus_backtab();
            } else {
                app.config_focus_tab();
            }
        }
        (_, KeyCode::BackTab) => app.config_focus_backtab(),
        (_, KeyCode::Down) => app.config_focus_next(),
        (_, KeyCode::Up) => app.config_focus_prev(),
        (_, KeyCode::Right) => app.config_focus_right(),
        (_, KeyCode::Left) => app.config_focus_left(),

        // Zenoh toggle
        (ConfigFocus::Zenoh, KeyCode::Char(' ') | KeyCode::Enter) => {
            if let Some(enabled) = app.zenoh().map(|c| c.enabled) {
                let result = app.client.set_enabled("zenoh", !enabled).await;
                app.apply_action(result);
            }
        }

        // Sim toggle; the orchestrator also switches the fieldname to the mode's default
        (ConfigFocus::Sim, KeyCode::Char(' ') | KeyCode::Enter) => {
            let update = ConfigUpdate {
                sim: Some(!app.sim()),
                ..Default::default()
            };
            let result = app.client.update_config(&update).await;
            app.apply_action(result);
            if let Some(status) = &app.status {
                app.config_fieldname_input = status.config.fieldname.clone();
            }
            app.config_focus = ConfigFocus::Zenoh;
        }

        // Fieldname text input, sent to the orchestrator on launch
        (ConfigFocus::Fieldname, KeyCode::Char(c)) => {
            if !modifiers.contains(KeyModifiers::CONTROL) {
                app.config_fieldname_input.push(c);
            }
        }
        (ConfigFocus::Fieldname, KeyCode::Backspace) => {
            app.config_fieldname_input.pop();
        }

        // DSD file text input, sent to the orchestrator on launch
        (ConfigFocus::DsdFile, KeyCode::Char(c)) => {
            if !modifiers.contains(KeyModifiers::CONTROL) {
                app.config_dsd_input.push(c);
            }
        }
        (ConfigFocus::DsdFile, KeyCode::Backspace) => {
            app.config_dsd_input.pop();
        }

        // Component checkbox toggle
        (ConfigFocus::Component(fi), KeyCode::Char(' ') | KeyCode::Enter) => {
            let toggleables = app.toggleable_indices();
            if let Some(&comp_idx) = toggleables.get(*fi) {
                let comp = &app.components()[comp_idx];
                let (key, enabled) = (comp.key.clone(), comp.enabled);
                let result = app.client.set_enabled(&key, !enabled).await;
                app.apply_action(result);
            }
        }

        // Start / Continue button
        (ConfigFocus::Start, KeyCode::Enter | KeyCode::Char(' ')) => {
            let update = ConfigUpdate {
                sim: None,
                fieldname: Some(app.config_fieldname_input.clone()),
                dsd_file: Some(app.config_dsd_input.clone()),
            };
            let result = app.client.update_config(&update).await;
            let ok = result.is_ok();
            app.apply_action(result);
            if ok {
                let result = app.client.start_all().await;
                app.apply_action(result);
                app.screen = Screen::Runtime;
            }
        }

        // Quit; the components keep running in the orchestrator
        (_, KeyCode::Char('q') | KeyCode::Char('Q') | KeyCode::Esc) => return true,

        _ => {}
    }
    false
}

pub async fn handle_runtime_key(app: &mut App, key: KeyCode, _modifiers: KeyModifiers) -> bool {
    let visible = app.runtime_visible_indices();
    let sel = app.runtime_selected.min(visible.len().saturating_sub(1));
    let selected = visible
        .get(sel)
        .map(|&idx| (idx, app.components()[idx].clone()));

    match key {
        KeyCode::Down | KeyCode::Char('j') => {
            app.runtime_selected = (sel + 1).min(visible.len().saturating_sub(1));
        }
        KeyCode::Up | KeyCode::Char('k') => {
            app.runtime_selected = sel.saturating_sub(1);
        }

        KeyCode::Char('s') => {
            if let Some((_, comp)) = selected {
                let result = if comp.state.is_alive() {
                    app.client.stop(&comp.key).await
                } else {
                    app.client.start(&comp.key).await
                };
                app.apply_action(result);
            }
        }

        KeyCode::Char('r') => {
            if let Some((_, comp)) = selected {
                let result = app.client.restart(&comp.key).await;
                app.apply_action(result);
            }
        }

        KeyCode::Char('l') => {
            if let Some((idx, _)) = selected {
                app.log_scroll = 0;
                app.log_follow = true;
                app.screen = Screen::Logs(idx);
            }
        }

        // Right: show sidebar if hidden, open AllLogs if sidebar already visible
        KeyCode::Right => {
            if app.show_log_panel {
                app.log_scroll = app.all_logs.lines.len().saturating_sub(1);
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

        KeyCode::Char('a') => {
            let result = app.client.start_all().await;
            app.apply_action(result);
        }
        KeyCode::Char('x') => app.confirm_action = Some(ConfirmAction::StopAll),
        KeyCode::Esc => app.screen = Screen::Config,
        KeyCode::Char('q') | KeyCode::Char('Q') => return true,
        _ => {}
    }
    false
}

pub fn handle_logs_key(app: &mut App, key: KeyCode) -> bool {
    let log_len = app.component_logs.lines.len();
    scroll_keys(app, key, log_len);
    match key {
        KeyCode::Esc | KeyCode::Char('q') | KeyCode::Char('l') => {
            app.screen = Screen::Runtime;
        }
        KeyCode::Char('Q') => return true,
        _ => {}
    }
    false
}

pub fn handle_all_logs_key(app: &mut App, key: KeyCode) -> bool {
    let log_len = app.all_logs.lines.len();
    scroll_keys(app, key, log_len);
    match key {
        KeyCode::Esc | KeyCode::Char('q') | KeyCode::Char('l') | KeyCode::Left => {
            app.screen = Screen::Runtime;
        }
        KeyCode::Char('Q') => return true,
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

pub async fn handle_confirm_key(app: &mut App, key: KeyCode) {
    match key {
        KeyCode::Char('y') | KeyCode::Char('Y') | KeyCode::Enter => {
            match app.confirm_action.take() {
                Some(ConfirmAction::StopAll) => {
                    let result = app.client.stop_all().await;
                    app.apply_action(result);
                }
                None => {}
            }
        }
        KeyCode::Char('n') | KeyCode::Char('N') | KeyCode::Esc => {
            app.confirm_action = None;
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::tests::offline_app;

    #[tokio::test]
    async fn confirm_dialog_only_reacts_to_labeled_keys() {
        let mut app = offline_app();
        app.confirm_action = Some(ConfirmAction::StopAll);
        for key in [KeyCode::Char('x'), KeyCode::Char(' '), KeyCode::Up] {
            handle_confirm_key(&mut app, key).await;
            assert!(app.confirm_action.is_some(), "{key:?} must not cancel");
        }
        for key in [KeyCode::Char('n'), KeyCode::Char('N'), KeyCode::Esc] {
            app.confirm_action = Some(ConfirmAction::StopAll);
            handle_confirm_key(&mut app, key).await;
            assert!(app.confirm_action.is_none(), "{key:?} must cancel");
        }
        for key in [KeyCode::Char('y'), KeyCode::Char('Y'), KeyCode::Enter] {
            app.confirm_action = Some(ConfirmAction::StopAll);
            handle_confirm_key(&mut app, key).await;
            assert!(app.confirm_action.is_none(), "{key:?} must confirm");
            // There is no orchestrator in this test, the refusal must be visible.
            assert!(app.action_error.is_some());
        }
    }
}
