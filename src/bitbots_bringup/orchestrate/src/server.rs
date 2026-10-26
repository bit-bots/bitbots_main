//! HTTP API of the orchestrator. All routes live below [`API_PREFIX`]; actions answer with
//! the resulting [`Status`].

use std::{
    fs::{File, OpenOptions, TryLockError},
    io::Write,
    os::unix::fs::DirBuilderExt,
    path::PathBuf,
    sync::{Arc, Mutex},
};

use anyhow::{bail, Context};

use axum::{
    extract::{Path, Query, State},
    http::StatusCode,
    response::{IntoResponse, Response},
    routing::{get, post, put},
    Json, Router,
};
use tokio::net::UnixListener;

use crate::{
    api::{lock_path, ApiError, ConfigUpdate, Logs, LogsQuery, SetEnabled, Status, API_PREFIX},
    components::component_idx,
    logs::LogStore,
    supervisor::{ActionError, Supervisor},
};

#[derive(Clone)]
pub struct ServerState {
    pub supervisor: Arc<Mutex<Supervisor>>,
    pub logs: Arc<LogStore>,
}

struct Failure(StatusCode, String);

impl IntoResponse for Failure {
    fn into_response(self) -> Response {
        (self.0, Json(ApiError { error: self.1 })).into_response()
    }
}

impl From<ActionError> for Failure {
    fn from(e: ActionError) -> Self {
        match e {
            ActionError::Conflict(msg) => Failure(StatusCode::CONFLICT, msg),
            ActionError::BadRequest(msg) => Failure(StatusCode::BAD_REQUEST, msg),
        }
    }
}

type ApiResult<T> = Result<Json<T>, Failure>;

fn lookup(key: &str) -> Result<usize, Failure> {
    component_idx(key).ok_or_else(|| {
        Failure(
            StatusCode::NOT_FOUND,
            format!("unknown component key '{key}'"),
        )
    })
}

/// Run `action` on the supervisor and answer with the resulting status.
fn act(
    state: &ServerState,
    action: impl FnOnce(&mut Supervisor) -> Result<(), ActionError>,
) -> ApiResult<Status> {
    let mut sup = state.supervisor.lock().unwrap();
    action(&mut sup)?;
    Ok(Json(sup.status()))
}

pub fn router(state: ServerState) -> Router {
    let api = Router::new()
        .route("/status", get(status))
        .route("/config", put(update_config))
        .route("/components/{key}/enabled", put(set_enabled))
        .route("/components/{key}/start", post(start))
        .route("/components/{key}/stop", post(stop))
        .route("/components/{key}/restart", post(restart))
        .route("/start_all", post(start_all))
        .route("/stop_all", post(stop_all))
        .route("/logs", get(logs));
    Router::new().nest(API_PREFIX, api).with_state(state)
}

async fn status(State(state): State<ServerState>) -> ApiResult<Status> {
    act(&state, |_| Ok(()))
}

async fn update_config(
    State(state): State<ServerState>,
    Json(update): Json<ConfigUpdate>,
) -> ApiResult<Status> {
    act(&state, |sup| sup.update_config(update))
}

async fn set_enabled(
    State(state): State<ServerState>,
    Path(key): Path<String>,
    Json(body): Json<SetEnabled>,
) -> ApiResult<Status> {
    let idx = lookup(&key)?;
    act(&state, |sup| {
        sup.set_enabled(idx, body.enabled);
        Ok(())
    })
}

async fn start(State(state): State<ServerState>, Path(key): Path<String>) -> ApiResult<Status> {
    let idx = lookup(&key)?;
    act(&state, |sup| sup.start_component(idx))
}

async fn stop(State(state): State<ServerState>, Path(key): Path<String>) -> ApiResult<Status> {
    let idx = lookup(&key)?;
    act(&state, |sup| {
        sup.stop_component(idx);
        Ok(())
    })
}

async fn restart(State(state): State<ServerState>, Path(key): Path<String>) -> ApiResult<Status> {
    let idx = lookup(&key)?;
    act(&state, |sup| sup.restart_component(idx))
}

async fn start_all(State(state): State<ServerState>) -> ApiResult<Status> {
    act(&state, |sup| sup.start_all_enabled())
}

async fn stop_all(State(state): State<ServerState>) -> ApiResult<Status> {
    act(&state, |sup| {
        sup.stop_all();
        Ok(())
    })
}

async fn logs(State(state): State<ServerState>, Query(query): Query<LogsQuery>) -> ApiResult<Logs> {
    if let Some(key) = &query.component {
        lookup(key)?;
    }
    Ok(Json(state.logs.since(
        query.since,
        query.component.as_deref(),
        query.limit.unwrap_or(usize::MAX),
    )))
}

/// Exclusive claim on a socket path, released when dropped.
pub struct SocketGuard {
    socket: PathBuf,
    _lock: File,
}

impl Drop for SocketGuard {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.socket);
    }
}

/// Bind the API socket, refusing to take over a socket that another instance still serves.
///
/// The claim is a lock on a file next to the socket, so a socket left behind by a crashed
/// instance is replaced while a live instance is never disturbed.
pub fn bind(socket: &std::path::Path) -> anyhow::Result<(UnixListener, SocketGuard)> {
    if let Some(dir) = socket.parent() {
        std::fs::DirBuilder::new()
            .recursive(true)
            .mode(0o700)
            .create(dir)
            .with_context(|| format!("cannot create socket directory {}", dir.display()))?;
    }
    let lock_path = lock_path(socket)?;
    let mut lock = OpenOptions::new()
        .create(true)
        .truncate(false)
        .read(true)
        .write(true)
        .open(&lock_path)
        .with_context(|| format!("cannot open lock file {}", lock_path.display()))?;
    match lock.try_lock() {
        Ok(()) => {}
        Err(TryLockError::WouldBlock) => {
            let owner = std::fs::read_to_string(&lock_path).unwrap_or_default();
            bail!(
                "another bitbots_orchestrate instance (pid {}) already serves {}",
                owner.trim(),
                socket.display()
            );
        }
        Err(TryLockError::Error(e)) => {
            return Err(e).with_context(|| format!("cannot lock {}", lock_path.display()))
        }
    }
    lock.set_len(0)?;
    write!(lock, "{}", std::process::id())?;

    match std::fs::remove_file(socket) {
        Ok(()) => eprintln!("removed stale socket {}", socket.display()),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
        Err(e) => {
            return Err(e)
                .with_context(|| format!("cannot remove stale socket {}", socket.display()))
        }
    }
    let listener =
        UnixListener::bind(socket).with_context(|| format!("cannot bind {}", socket.display()))?;
    Ok((
        listener,
        SocketGuard {
            socket: socket.to_path_buf(),
            _lock: lock,
        },
    ))
}
