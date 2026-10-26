//! Starting an orchestrator for the TUI when none is running.

use std::{
    fs::File,
    os::unix::process::CommandExt,
    path::{Path, PathBuf},
};

use anyhow::{bail, Context, Result};
use tokio::process::{Child, Command};

/// `bitbots_orchestrate`, installed next to this executable.
fn orchestrator_exe() -> Result<PathBuf> {
    let exe = std::env::current_exe()
        .context("cannot locate the teamplayer executable")?
        .with_file_name("bitbots_orchestrate");
    if !exe.exists() {
        bail!(
            "bitbots_orchestrate not found next to teamplayer at {}",
            exe.display()
        );
    }
    Ok(exe)
}

/// Replace this process with an orchestrator in the foreground, serving `socket`.
///
/// It stays in the terminal's process group, so Ctrl-C stops it together with all components.
/// Only returns if the exec failed.
pub fn exec_headless(socket: &Path, args: &[String]) -> anyhow::Error {
    let exe = match orchestrator_exe() {
        Ok(exe) => exe,
        Err(e) => return e,
    };
    let err = std::process::Command::new(&exe)
        .arg("--socket")
        .arg(socket)
        .args(args)
        .exec();
    anyhow::Error::new(err).context(format!("cannot run {}", exe.display()))
}

/// An orchestrator process started by this TUI.
pub struct OwnedOrchestrator {
    child: Child,
    pub log_path: PathBuf,
}

impl OwnedOrchestrator {
    #[cfg(test)]
    pub fn from_child(child: Child) -> Self {
        Self {
            child,
            log_path: PathBuf::from("/dev/null"),
        }
    }

    /// Start `bitbots_orchestrate` in the background, serving `socket`.
    ///
    /// It runs in its own session so it survives the TUI and its terminal; its output goes to a
    /// log file next to the socket.
    pub fn spawn(socket: &Path, args: &[String]) -> Result<Self> {
        let exe = orchestrator_exe()?;
        if let Some(dir) = socket.parent() {
            std::fs::create_dir_all(dir)
                .with_context(|| format!("cannot create {}", dir.display()))?;
        }
        let log_path = socket.with_extension("log");
        let log = File::create(&log_path)
            .with_context(|| format!("cannot create {}", log_path.display()))?;

        let mut cmd = std::process::Command::new(&exe);
        cmd.arg("--socket")
            .arg(socket)
            .args(args)
            .stdin(std::process::Stdio::null())
            .stdout(log.try_clone()?)
            .stderr(log);
        // SAFETY: setsid is async-signal-safe and touches no state of the parent.
        unsafe {
            cmd.pre_exec(|| nix::unistd::setsid().map(|_| ()).map_err(Into::into));
        }
        let child = Command::from(cmd)
            .spawn()
            .with_context(|| format!("cannot start {}", exe.display()))?;
        Ok(Self { child, log_path })
    }

    /// Exit status, if the orchestrator has already exited.
    pub fn try_exited(&mut self) -> Option<std::process::ExitStatus> {
        self.child.try_wait().ok().flatten()
    }

    /// Ask the orchestrator to stop all components and itself, and wait until it is gone.
    pub async fn stop(mut self) -> Result<std::process::ExitStatus> {
        if let Some(pid) = self.child.id() {
            use nix::{sys::signal::kill, sys::signal::Signal, unistd::Pid};
            kill(Pid::from_raw(pid as i32), Signal::SIGTERM)
                .context("cannot signal the orchestrator")?;
        }
        Ok(self.child.wait().await?)
    }
}
