//! Supervisor for the processes of the robot software stack.
//!
//! The `bitbots_orchestrate` binary runs a [`supervisor::Supervisor`] and serves it through an
//! HTTP API on a Unix socket (see [`server`]). Frontends such as the teamplayer TUI talk to it
//! through [`client::Client`].

pub mod api;
pub mod client;
pub mod logs;
pub mod server;
pub mod supervisor;
