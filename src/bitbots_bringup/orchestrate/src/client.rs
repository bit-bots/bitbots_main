//! Client for the HTTP API of a local orchestrate instance.

use std::path::{Path, PathBuf};

use anyhow::{bail, Context, Result};
use http_body_util::{BodyExt, Full};
use hyper::{
    body::Bytes,
    header::{CONTENT_TYPE, HOST},
    Method, Request,
};
use hyper_util::rt::TokioIo;
use serde::{de::DeserializeOwned, Serialize};
use tokio::net::UnixStream;

use crate::api::{ApiError, ConfigUpdate, Logs, SetEnabled, Status, API_PREFIX};

#[derive(Clone)]
pub struct Client {
    socket: PathBuf,
}

impl Client {
    pub fn new(socket: PathBuf) -> Self {
        Self { socket }
    }

    pub fn socket(&self) -> &Path {
        &self.socket
    }

    async fn request<T: DeserializeOwned>(
        &self,
        method: Method,
        path: &str,
        body: Option<&impl Serialize>,
    ) -> Result<T> {
        let stream = UnixStream::connect(&self.socket).await.with_context(|| {
            format!(
                "cannot connect to bitbots_orchestrate at {}",
                self.socket.display()
            )
        })?;
        let (mut sender, conn) = hyper::client::conn::http1::handshake(TokioIo::new(stream))
            .await
            .context("HTTP handshake with bitbots_orchestrate failed")?;
        // Connection errors surface through send_request below.
        tokio::spawn(conn);

        let body = match body {
            Some(b) => Bytes::from(serde_json::to_vec(b)?),
            None => Bytes::new(),
        };
        let request = Request::builder()
            .method(method.clone())
            .uri(format!("{API_PREFIX}{path}"))
            .header(HOST, "localhost")
            .header(CONTENT_TYPE, "application/json")
            .body(Full::new(body))?;
        let response = sender
            .send_request(request)
            .await
            .with_context(|| format!("{method} {path} failed"))?;
        let status = response.status();
        let bytes = response.into_body().collect().await?.to_bytes();
        if !status.is_success() {
            let msg = serde_json::from_slice::<ApiError>(&bytes)
                .map(|e| e.error)
                .unwrap_or_else(|_| String::from_utf8_lossy(&bytes).into_owned());
            bail!("{method} {path}: {status}: {msg}");
        }
        serde_json::from_slice(&bytes)
            .with_context(|| format!("{method} {path}: malformed response"))
    }

    pub async fn status(&self) -> Result<Status> {
        self.request(Method::GET, "/status", None::<&()>).await
    }

    pub async fn update_config(&self, update: &ConfigUpdate) -> Result<Status> {
        self.request(Method::PUT, "/config", Some(update)).await
    }

    pub async fn set_enabled(&self, key: &str, enabled: bool) -> Result<Status> {
        let path = format!("/components/{key}/enabled");
        self.request(Method::PUT, &path, Some(&SetEnabled { enabled }))
            .await
    }

    async fn component_action(&self, key: &str, action: &str) -> Result<Status> {
        let path = format!("/components/{key}/{action}");
        self.request(Method::POST, &path, None::<&()>).await
    }

    pub async fn start(&self, key: &str) -> Result<Status> {
        self.component_action(key, "start").await
    }

    pub async fn stop(&self, key: &str) -> Result<Status> {
        self.component_action(key, "stop").await
    }

    pub async fn restart(&self, key: &str) -> Result<Status> {
        self.component_action(key, "restart").await
    }

    pub async fn start_all(&self) -> Result<Status> {
        self.request(Method::POST, "/start_all", None::<&()>).await
    }

    pub async fn stop_all(&self) -> Result<Status> {
        self.request(Method::POST, "/stop_all", None::<&()>).await
    }

    /// Log lines newer than `since`, optionally of one component only.
    pub async fn logs(&self, since: u64, component: Option<&str>) -> Result<Logs> {
        let mut path = format!("/logs?since={since}");
        if let Some(key) = component {
            path.push_str(&format!("&component={key}"));
        }
        self.request(Method::GET, &path, None::<&()>).await
    }
}
