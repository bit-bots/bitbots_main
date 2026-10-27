//! Starting a compose server for the TUI when none is running.

use std::{
    fs::File,
    os::unix::process::CommandExt,
    path::{Path, PathBuf},
};

use anyhow::{bail, Context, Result};
use tokio::process::{Child, Command};

/// `bitbots_compose_server`, installed next to this executable.
fn compose_server_exe() -> Result<PathBuf> {
    let exe = std::env::current_exe()
        .context("cannot locate the TUI executable")?
        .with_file_name("bitbots_compose_server");
    if !exe.exists() {
        bail!(
            "bitbots_compose_server not found next to the TUI at {}",
            exe.display()
        );
    }
    Ok(exe)
}

/// Replace this process with a compose server in the foreground, serving `socket`.
///
/// It stays in the terminal's process group, so Ctrl-C stops it together with all components.
/// Only returns if the exec failed.
pub fn exec_headless(socket: &Path, args: &[String]) -> anyhow::Error {
    let exe = match compose_server_exe() {
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

/// A compose server process started by this TUI.
pub struct OwnedComposeServer {
    child: Child,
    pub log_path: PathBuf,
}

impl OwnedComposeServer {
    #[cfg(test)]
    pub fn from_child(child: Child) -> Self {
        Self {
            child,
            log_path: PathBuf::from("/dev/null"),
        }
    }

    /// Start `bitbots_compose_server` in the background, serving `socket`.
    ///
    /// It runs in its own session so it survives the TUI and its terminal; its output goes to a
    /// log file next to the socket.
    pub fn spawn(socket: &Path, args: &[String]) -> Result<Self> {
        let exe = compose_server_exe()?;
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

    /// Exit status, if the compose server has already exited.
    pub fn try_exited(&mut self) -> Option<std::process::ExitStatus> {
        self.child.try_wait().ok().flatten()
    }

    /// Wait until the compose server has exited, e.g. after a shutdown request through the API.
    pub async fn wait(mut self) -> Result<std::process::ExitStatus> {
        Ok(self.child.wait().await?)
    }

    /// Ask the compose server to stop all components and itself, and wait until it is gone.
    pub async fn stop(mut self) -> Result<std::process::ExitStatus> {
        if let Some(pid) = self.child.id() {
            use nix::{sys::signal::kill, sys::signal::Signal, unistd::Pid};
            kill(Pid::from_raw(pid as i32), Signal::SIGTERM)
                .context("cannot signal the compose server")?;
        }
        Ok(self.child.wait().await?)
    }
}
