use std::{
    path::PathBuf,
    sync::{Arc, Mutex},
    time::Duration,
};

use anyhow::Result;
use clap::Parser;
use tokio::{
    signal::unix::{signal, SignalKind},
    time,
};

use bitbots_orchestrate::{
    api::{resolve_socket, LogStream, DEFAULT_INSTANCE_NAME},
    components::DSD_FILE_DEFAULT,
    logs::LogStore,
    server::{self, ServerState},
    supervisor::Supervisor,
};

/// How long components get to exit after SIGTERM on shutdown before they are killed.
const SHUTDOWN_TIMEOUT: Duration = Duration::from_secs(10);

#[derive(Parser)]
#[command(
    name = "bitbots_orchestrate",
    about = "Bit-Bots process orchestrator — runs the robot software components and serves an \
             HTTP API on a Unix socket for frontends such as the teamplayer TUI",
    long_about = None,
)]
struct Cli {
    /// Start all enabled components right away instead of waiting for a frontend
    #[arg(long)]
    autostart: bool,

    /// Enable simulation mode (also enables the simulator, disables hardware-only components)
    #[arg(long)]
    sim: bool,

    /// Override the field name [default: labor, or hsl_kid in sim]
    #[arg(long, value_name = "NAME")]
    fieldname: Option<String>,

    /// Override the behavior DSD file
    #[arg(long, value_name = "FILE", default_value = DSD_FILE_DEFAULT)]
    dsd_file: String,

    /// Disable the Zenoh daemon (useful when zenoh is already running externally)
    #[arg(long)]
    no_zenoh: bool,

    /// Enable a component by key; may be specified multiple times
    #[arg(long = "enable", value_name = "KEY")]
    enable: Vec<String>,

    /// Disable a component by key; may be specified multiple times
    #[arg(long = "disable", value_name = "KEY")]
    disable: Vec<String>,

    /// Instance name, selects the socket so several instances can run on one machine
    #[arg(long, default_value = DEFAULT_INSTANCE_NAME)]
    name: String,

    /// Explicit socket path, overrides --name
    #[arg(long, value_name = "PATH")]
    socket: Option<PathBuf>,
}

#[tokio::main]
async fn main() -> Result<()> {
    let cli = Cli::parse();

    let logs = Arc::new(LogStore::new());
    let mut supervisor = Supervisor::new(logs.clone());
    supervisor.apply_cli_presets(
        cli.sim,
        cli.no_zenoh,
        cli.fieldname.as_deref(),
        &cli.dsd_file,
        &cli.enable,
        &cli.disable,
    )?;

    let socket = resolve_socket(cli.socket, &cli.name)?;
    let (listener, _guard) = server::bind(&socket)?;
    eprintln!("bitbots_orchestrate listening on {}", socket.display());

    if cli.autostart {
        if let Err(e) = supervisor.start_all_enabled() {
            eprintln!("autostart failed: {e}");
        }
    }
    let supervisor = Arc::new(Mutex::new(supervisor));
    let router = server::router(ServerState {
        supervisor: supervisor.clone(),
        logs: logs.clone(),
    });
    let server = tokio::spawn(async move { axum::serve(listener, router).await });

    let mut sigterm = signal(SignalKind::terminate())?;
    let mut printed_seq = 0;
    let mut tick = time::interval(Duration::from_millis(50));
    loop {
        tokio::select! {
            _ = tick.tick() => {
                supervisor.lock().unwrap().tick();
                printed_seq = print_logs(&logs, printed_seq);
                if server.is_finished() {
                    eprintln!("API server stopped unexpectedly, shutting down");
                    break;
                }
            }
            _ = tokio::signal::ctrl_c() => break,
            _ = sigterm.recv() => break,
        }
    }

    eprintln!("stopping all components...");
    supervisor.lock().unwrap().begin_shutdown();
    let deadline = time::Instant::now() + SHUTDOWN_TIMEOUT;
    while supervisor.lock().unwrap().any_alive() && time::Instant::now() < deadline {
        tick.tick().await;
        supervisor.lock().unwrap().tick();
        printed_seq = print_logs(&logs, printed_seq);
    }
    if supervisor.lock().unwrap().any_alive() {
        supervisor.lock().unwrap().kill_remaining();
        // Give the exit reports of the killed processes a moment to arrive.
        time::sleep(Duration::from_millis(200)).await;
        supervisor.lock().unwrap().tick();
    }
    print_logs(&logs, printed_seq);
    server.abort();
    Ok(())
}

/// Print all log lines newer than `since` and return the newest printed sequence number.
fn print_logs(logs: &LogStore, since: u64) -> u64 {
    let batch = logs.since(since, None, usize::MAX);
    if batch.truncated {
        eprintln!("[orchestrate] log output fell behind, some lines were dropped");
    }
    let mut last = since;
    for line in batch.lines {
        let text = format!("[{}] {}", line.component_name, line.text);
        match line.stream {
            LogStream::Stdout => println!("{text}"),
            LogStream::Stderr => eprintln!("{text}"),
        }
        last = line.seq;
    }
    last
}
