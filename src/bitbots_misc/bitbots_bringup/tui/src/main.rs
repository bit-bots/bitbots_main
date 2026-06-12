mod app;
mod components;
mod control;
mod input;
mod ui;

use anyhow::Result;
use clap::{Args, Parser, Subcommand};
use crossterm::{
    event::{DisableMouseCapture, EnableMouseCapture, Event, EventStream},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use futures::StreamExt;
use ratatui::{backend::CrosstermBackend, Terminal};
use std::time::Duration;
use tokio::sync::{broadcast, mpsc};
use tokio::time;

use app::{App, ProcState, Screen};
use components::COMPONENT_DEFS;

// ─── CLI ──────────────────────────────────────────────────────────────────────

#[derive(Parser)]
#[command(
    name = "teamplayer",
    about = "Bit-Bots teamplayer launcher — interactive TUI or headless mode",
    long_about = None,
)]
struct Cli {
    #[command(subcommand)]
    command: Option<Command>,

    /// Run without TUI: start all enabled components and block until Ctrl-C
    #[arg(long)]
    headless: bool,

    /// Enable simulation mode (also enables the simulator, disables hardware-only components)
    #[arg(long)]
    sim: bool,

    /// Override the field name [default: labor, or hsl_kid in sim]
    #[arg(long, value_name = "NAME")]
    fieldname: Option<String>,

    /// Override the behavior DSD file [default: main.dsd]
    #[arg(long, value_name = "FILE", default_value = "main.dsd")]
    dsd_file: String,

    /// Runtime game-settings YAML loaded by the global parameter blackboard
    #[arg(long, value_name = "PATH")]
    game_settings: Option<String>,

    /// Disable the Zenoh daemon (useful when zenoh is already running externally)
    #[arg(long)]
    no_zenoh: bool,

    /// Enable a component by key; may be specified multiple times.
    /// Available keys: zenoh, blackboard, robot_description, diagnostics,
    /// simulator, lowlevel, motion, game_controller, vision, ipm, localization,
    /// path_planning, behavior, teamcom, world_model, whistle_detector, audio,
    /// tts, monitoring, record
    #[arg(long = "enable", value_name = "KEY")]
    enable: Vec<String>,

    /// Disable a component by key; may be specified multiple times
    #[arg(long = "disable", value_name = "KEY")]
    disable: Vec<String>,
}

#[derive(Subcommand)]
enum Command {
    /// Control a running teamplayer instance
    Ctl(ControlArgs),
}

#[derive(Args)]
struct ControlArgs {
    #[command(subcommand)]
    command: ControlSubcommand,
}

#[derive(Subcommand)]
enum ControlSubcommand {
    Status {
        #[arg(long)]
        json: bool,
    },
    Watch {
        #[arg(long)]
        jsonl: bool,
    },
    SetComponents {
        #[arg(long = "enable")]
        enable: Vec<String>,
    },
    SetConfig {
        #[arg(long)]
        fieldname: String,
        #[arg(long)]
        game_settings: Option<String>,
    },
    Start {
        #[arg(default_value = "all")]
        target: String,
    },
    Stop {
        #[arg(default_value = "all")]
        target: String,
    },
    Restart {
        #[arg(default_value = "all")]
        target: String,
    },
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

    if let Some(Command::Ctl(args)) = &cli.command {
        return run_control_client(args).await;
    }

    let (control_tx, control_rx) = mpsc::unbounded_channel();
    let (event_tx, _) = broadcast::channel(2048);
    let control_path = control::socket_path();
    let control_server =
        control::start_server(control_path.clone(), control_tx, event_tx.clone()).await?;

    let mut app = App::new(control_rx, event_tx);
    app.apply_cli_presets(
        cli.sim,
        cli.no_zenoh,
        cli.fieldname.as_deref(),
        &cli.dsd_file,
        cli.game_settings.as_deref(),
        &cli.enable,
        &cli.disable,
    );

    let result = if cli.headless {
        run_headless(&mut app).await
    } else {
        setup_panic_hook();
        enable_raw_mode()?;
        let mut stdout = std::io::stdout();
        execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
        let backend = CrosstermBackend::new(stdout);
        let mut terminal = Terminal::new(backend)?;

        let mut event_stream = EventStream::new();
        let mut tick_interval = time::interval(Duration::from_millis(100));
        let r = run_tui(
            &mut terminal,
            &mut app,
            &mut event_stream,
            &mut tick_interval,
        )
        .await;
        cleanup_terminal();
        r
    };

    // SIGTERM all remaining process groups on exit
    {
        use nix::sys::signal::{killpg, Signal};
        use nix::unistd::Pid;
        for comp in &app.components {
            for &pid in &comp.pids {
                let _ = killpg(Pid::from_raw(pid as i32), Signal::SIGTERM);
            }
        }
    }
    control_server.abort();
    let _ = std::fs::remove_file(control_path);

    result
}

async fn run_control_client(args: &ControlArgs) -> Result<()> {
    use control::ControlRequest;
    let (request, watch) = match &args.command {
        ControlSubcommand::Status { .. } => (ControlRequest::Status, false),
        ControlSubcommand::Watch { .. } => (ControlRequest::Watch, true),
        ControlSubcommand::SetComponents { enable } => (
            ControlRequest::SetComponents {
                components: enable.clone(),
            },
            false,
        ),
        ControlSubcommand::SetConfig {
            fieldname,
            game_settings,
        } => (
            ControlRequest::SetConfig {
                fieldname: fieldname.clone(),
                game_settings: game_settings.clone(),
            },
            false,
        ),
        ControlSubcommand::Start { target } => (
            ControlRequest::Start {
                target: target.clone(),
            },
            false,
        ),
        ControlSubcommand::Stop { target } => (
            ControlRequest::Stop {
                target: target.clone(),
            },
            false,
        ),
        ControlSubcommand::Restart { target } => (
            ControlRequest::Restart {
                target: target.clone(),
            },
            false,
        ),
    };
    control::request(&control::socket_path(), &request, watch).await
}

// ─── Headless mode ────────────────────────────────────────────────────────────

async fn run_headless(app: &mut App) -> Result<()> {
    eprintln!("teamplayer [headless] — Ctrl-C to stop");
    eprintln!();

    // Print what will be started
    for comp in &app.components {
        let def = &COMPONENT_DEFS[comp.comp_idx];
        if comp.enabled {
            eprintln!("  [+] {}", def.name);
        }
    }
    eprintln!();

    app.start_all_enabled().await;

    let mut tick = time::interval(Duration::from_millis(50));
    loop {
        tokio::select! {
            _ = tick.tick() => {
                let to_restart = app.drain_messages();
                for idx in to_restart {
                    app.start_component(idx).await;
                }
                app.drain_control_commands().await;
                {
                    let mut q = app.headless_log.lock().unwrap();
                    while let Some(entry) = q.pop_front() {
                        match entry {
                            app::HeadlessLine::Stdout(s) => println!("{s}"),
                            app::HeadlessLine::Stderr(s) => eprintln!("{s}"),
                        }
                    }
                }
                let any_running = app.components.iter().any(|c| {
                    matches!(c.state, ProcState::Running | ProcState::Stopping | ProcState::Restarting)
                });
                if !any_running {
                    eprintln!("All components have stopped.");
                    break;
                }
            }
            _ = tokio::signal::ctrl_c() => {
                eprintln!("\nInterrupted — stopping all components...");
                break;
            }
        }
    }
    Ok(())
}

// ─── TUI event loop ───────────────────────────────────────────────────────────

async fn run_tui(
    terminal: &mut Terminal<CrosstermBackend<std::io::Stdout>>,
    app: &mut App,
    event_stream: &mut EventStream,
    tick_interval: &mut time::Interval,
) -> Result<()> {
    loop {
        terminal.draw(|f| ui::draw(f, app))?;

        tokio::select! {
            _ = tick_interval.tick() => {
                app.tick_count += 1;
                let to_restart = app.drain_messages();
                for idx in to_restart {
                    app.start_component(idx).await;
                }
                app.drain_control_commands().await;
                if app.log_follow {
                    let len = match &app.screen {
                        Screen::Logs(idx) => app.components[*idx].logs.lock().unwrap().len(),
                        Screen::AllLogs => app.all_logs.lock().unwrap().len(),
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
                    let quit = if app.confirm_action.is_some() {
                        input::handle_confirm_key(app, key_event.code).await
                    } else {
                        match app.screen.clone() {
                            Screen::Config => {
                                input::handle_config_key(app, key_event.code, key_event.modifiers).await
                            }
                            Screen::Runtime => {
                                input::handle_runtime_key(app, key_event.code, key_event.modifiers).await
                            }
                            Screen::Logs(idx) => {
                                input::handle_logs_key(app, key_event.code, key_event.modifiers, idx).await
                            }
                            Screen::AllLogs => {
                                input::handle_all_logs_key(app, key_event.code)
                            }
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
