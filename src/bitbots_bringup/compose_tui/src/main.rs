mod app;
mod input;
mod spawn;
mod ui;

use std::{path::PathBuf, time::Duration};

use anyhow::{bail, Result};
use bitbots_compose_api::{
    api::{lock_owner, resolve_socket, DEFAULT_INSTANCE_NAME},
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

use app::{App, ExitMode, Screen};
use spawn::OwnedComposeServer;

// ─── CLI ──────────────────────────────────────────────────────────────────────

#[derive(Parser)]
#[command(
    name = "bitbots_compose_tui",
    about = "Bit-Bots teamplayer TUI — frontend for bitbots_compose_server, starts one if none is \
             running",
    long_about = None,
)]
struct Cli {
    /// Run without TUI: run a compose server in the foreground that starts all enabled
    /// components right away; Ctrl-C stops them together with the compose server
    #[arg(long, conflicts_with = "attach")]
    headless: bool,

    /// Only connect to a running compose server, never start one
    #[arg(long)]
    attach: bool,

    /// Settings for a newly started compose server, see `bitbots_compose_server --help`.
    /// Refused when attaching to a running compose server.
    #[command(flatten)]
    presets: Presets,

    /// Name of the compose server instance to connect to
    #[arg(long, default_value = DEFAULT_INSTANCE_NAME)]
    name: String,

    /// Explicit socket path of the compose server instance, overrides --name
    #[arg(long, value_name = "PATH")]
    socket: Option<PathBuf>,
}

#[derive(clap::Args)]
struct Presets {
    /// Start all enabled components right away
    #[arg(long)]
    autostart: bool,

    /// Compose file declaring the components
    #[arg(long, value_name = "PATH")]
    file: Option<PathBuf>,

    /// Apply a profile of the compose file, e.g. sim
    #[arg(long, value_name = "NAME")]
    profile: Option<String>,

    /// Set a parameter of the compose file; may be repeated
    #[arg(long = "set", value_name = "NAME=VALUE")]
    set: Vec<String>,

    /// Enable a component by key; may be repeated
    #[arg(long = "enable", value_name = "KEY")]
    enable: Vec<String>,

    /// Disable a component by key; may be repeated
    #[arg(long = "disable", value_name = "KEY")]
    disable: Vec<String>,

    /// Override a global parameter, e.g. bot_id=2; may be repeated
    #[arg(long = "global", value_name = "NAME=VALUE")]
    global: Vec<String>,
}

impl Presets {
    /// Command line arguments for bitbots_compose_server.
    fn to_args(&self) -> Vec<String> {
        let mut args = Vec::new();
        if self.autostart {
            args.push("--autostart".to_string());
        }
        if let Some(file) = &self.file {
            args.extend(["--file".to_string(), file.display().to_string()]);
        }
        if let Some(profile) = &self.profile {
            args.extend(["--profile".to_string(), profile.clone()]);
        }
        for (values, flag) in [
            (&self.set, "--set"),
            (&self.enable, "--enable"),
            (&self.disable, "--disable"),
            (&self.global, "--global"),
        ] {
            for value in values {
                args.extend([flag.to_string(), value.clone()]);
            }
        }
        args
    }
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

fn setup_terminal() -> Result<Terminal<CrosstermBackend<std::io::Stdout>>> {
    enable_raw_mode()?;
    let mut stdout = std::io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
    Ok(Terminal::new(CrosstermBackend::new(stdout))?)
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
    let client = Client::new(socket.clone());
    let preset_args = cli.presets.to_args();

    if cli.headless {
        let mut args = preset_args;
        if !cli.presets.autostart {
            args.push("--autostart".to_string());
        }
        return Err(spawn::exec_headless(&socket, &args));
    }

    // Only start a compose server if nothing listens on the socket; one that listens but does
    // not answer as expected must not be mistaken for a missing one.
    let running = client.is_listening().await;
    let compose_server = if running {
        if let Err(e) = client.status().await {
            let owner = lock_owner(&socket).map_or("unknown".to_string(), |pid| pid.to_string());
            bail!(
                "a compose server (pid {owner}) listens on {} but does not answer as expected, \
                 probably one of an older build: {e:#}\nStop it with `kill -TERM {owner}` and \
                 start the TUI again",
                socket.display()
            );
        }
        if !preset_args.is_empty() {
            bail!(
                "a compose server is already running at {}, its settings cannot be changed from \
                 the command line; drop {} or stop it first",
                socket.display(),
                preset_args.join(" ")
            );
        }
        None
    } else if cli.attach {
        if !preset_args.is_empty() {
            bail!("--attach does not start a compose server, so the presets would have no effect");
        }
        None
    } else {
        Some(OwnedComposeServer::spawn(&socket, &preset_args)?)
    };

    let mut app = App::new(client);
    app.compose_server = compose_server;

    setup_panic_hook();
    let result = match setup_terminal() {
        Ok(mut terminal) => run_tui(&mut terminal, &mut app).await,
        Err(e) => Err(e),
    };
    cleanup_terminal();
    let exit = match result {
        Ok(exit) => exit,
        Err(e) => {
            // Do not leave a compose server behind that nobody knows about.
            if let Some(compose_server) = app.compose_server.take() {
                eprintln!("Stopping the compose server started by the TUI...");
                let _ = compose_server.stop().await;
            }
            return Err(e);
        }
    };

    match exit {
        ExitMode::StopComposeServer => {
            eprintln!("Stopping all components and the compose server...");
            stop_compose_server(&app.client, app.compose_server.take()).await?;
        }
        ExitMode::Detach if app.status.is_some() => {
            let pid = lock_owner(&socket).map_or("?".to_string(), |pid| pid.to_string());
            let log = match &app.compose_server {
                Some(compose_server) => format!(", log: {}", compose_server.log_path.display()),
                None => String::new(),
            };
            eprintln!(
                "The compose server keeps running in the background (pid {pid}{log}).\n\
                 Reattach with `pixi run teamplayer --name {}`.",
                cli.name,
            );
        }
        ExitMode::Detach => {}
    }
    Ok(())
}

/// How long to wait for a compose server to stop. It stops its components with a timeout of its
/// own and kills what remains, so this only runs out if it hangs.
const STOP_WAIT: Duration = Duration::from_secs(30);

/// Stop the compose server through its API and wait until it is gone. One that this TUI started
/// is sent SIGTERM if the API does not answer.
async fn stop_compose_server(client: &Client, owned: Option<OwnedComposeServer>) -> Result<()> {
    let requested = client.shutdown().await;
    if let Some(compose_server) = owned {
        let log_path = compose_server.log_path.clone();
        let status = match requested {
            Ok(_) => compose_server.wait().await?,
            Err(e) => {
                eprintln!("The shutdown request failed ({e:#}), sending SIGTERM instead");
                compose_server.stop().await?
            }
        };
        if !status.success() {
            bail!(
                "the compose server exited with {status}, see {}",
                log_path.display()
            );
        }
        return Ok(());
    }
    requested.map_err(|e| e.context("cannot ask the compose server to shut down"))?;
    let deadline = time::Instant::now() + STOP_WAIT;
    while client.is_listening().await {
        if time::Instant::now() > deadline {
            let pid = lock_owner(client.socket()).map_or("?".to_string(), |pid| pid.to_string());
            bail!("the compose server (pid {pid}) is still running after {STOP_WAIT:?}");
        }
        time::sleep(Duration::from_millis(200)).await;
    }
    Ok(())
}

// ─── TUI event loop ───────────────────────────────────────────────────────────

async fn run_tui(
    terminal: &mut Terminal<CrosstermBackend<std::io::Stdout>>,
    app: &mut App,
) -> Result<ExitMode> {
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
                    } else {
                        match app.screen.clone() {
                            Screen::Config => {
                                input::handle_config_key(app, key_event.code, key_event.modifiers)
                                    .await
                            }
                            Screen::Runtime => input::handle_runtime_key(app, key_event.code).await,
                            Screen::Logs(_) => input::handle_logs_key(app, key_event.code),
                            Screen::AllLogs => input::handle_all_logs_key(app, key_event.code),
                        }
                    }
                    if let Some(exit) = app.exit.take() {
                        return Ok(exit);
                    }
                }
            }

            _ = tokio::signal::ctrl_c() => {
                return Ok(ExitMode::Detach);
            }
        }
    }
    Ok(ExitMode::Detach)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_are_passed_on_to_the_compose_server() {
        let cli = Cli::parse_from([
            "bitbots_compose_tui",
            "--profile",
            "sim",
            "--global",
            "bot_id=2",
            "--global",
            "role=goalie",
        ]);
        assert_eq!(
            cli.presets.to_args(),
            [
                "--profile",
                "sim",
                "--global",
                "bot_id=2",
                "--global",
                "role=goalie"
            ]
        );
    }
}
