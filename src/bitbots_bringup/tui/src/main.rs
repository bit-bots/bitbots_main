mod app;
mod input;
mod ui;

use std::{path::PathBuf, time::Duration};

use anyhow::Result;
use bitbots_orchestrate::{
    api::{resolve_socket, DEFAULT_INSTANCE_NAME},
    client::Client,
};
use clap::Parser;
use crossterm::{
    event::{DisableMouseCapture, EnableMouseCapture, Event, EventStream},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use futures::StreamExt;
use ratatui::{backend::CrosstermBackend, Terminal};
use tokio::time;

use app::{App, Screen};

// ─── CLI ──────────────────────────────────────────────────────────────────────

#[derive(Parser)]
#[command(
    name = "teamplayer",
    about = "Bit-Bots teamplayer TUI — frontend for a local bitbots_orchestrate instance",
    long_about = None,
)]
struct Cli {
    /// Name of the orchestrate instance to connect to
    #[arg(long, default_value = DEFAULT_INSTANCE_NAME)]
    name: String,

    /// Explicit socket path of the orchestrate instance, overrides --name
    #[arg(long, value_name = "PATH")]
    socket: Option<PathBuf>,
}

// ─── Terminal helpers ─────────────────────────────────────────────────────────

fn setup_panic_hook() {
    let original = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let _ = disable_raw_mode();
        let _ = execute!(std::io::stdout(), LeaveAlternateScreen, DisableMouseCapture);
        original(info);
    }));
}

fn cleanup_terminal() {
    let _ = disable_raw_mode();
    let _ = execute!(std::io::stdout(), LeaveAlternateScreen, DisableMouseCapture);
}

// ─── Entry point ──────────────────────────────────────────────────────────────

#[tokio::main]
async fn main() -> Result<()> {
    let cli = Cli::parse();
    let socket = resolve_socket(cli.socket, &cli.name)?;
    let mut app = App::new(Client::new(socket));

    setup_panic_hook();
    enable_raw_mode()?;
    let mut stdout = std::io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let result = run_tui(&mut terminal, &mut app).await;
    cleanup_terminal();
    result
}

// ─── TUI event loop ───────────────────────────────────────────────────────────

async fn run_tui(
    terminal: &mut Terminal<CrosstermBackend<std::io::Stdout>>,
    app: &mut App,
) -> Result<()> {
    let mut event_stream = EventStream::new();
    let mut tick_interval = time::interval(Duration::from_millis(100));
    loop {
        terminal.draw(|f| ui::draw(f, app))?;

        tokio::select! {
            _ = tick_interval.tick() => {
                app.tick_count += 1;
                app.refresh().await;
                if app.log_follow {
                    let len = match &app.screen {
                        Screen::Logs(_) => app.component_logs.lines.len(),
                        Screen::AllLogs => app.all_logs.lines.len(),
                        _ => 0,
                    };
                    if len > 0 {
                        app.log_scroll = len.saturating_sub(1);
                    }
                }
            }

            maybe_event = event_stream.next() => {
                let Some(Ok(event)) = maybe_event else { break };
                if let Event::Key(key_event) = event {
                    if app.confirm_action.is_some() {
                        input::handle_confirm_key(app, key_event.code).await;
                        continue;
                    }
                    let quit = match app.screen.clone() {
                        Screen::Config => {
                            input::handle_config_key(app, key_event.code, key_event.modifiers).await
                        }
                        Screen::Runtime => {
                            input::handle_runtime_key(app, key_event.code, key_event.modifiers).await
                        }
                        Screen::Logs(_) => input::handle_logs_key(app, key_event.code),
                        Screen::AllLogs => input::handle_all_logs_key(app, key_event.code),
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
