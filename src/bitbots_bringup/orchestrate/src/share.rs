//! Locating files of installed ROS packages.

use std::path::{Path, PathBuf};

use anyhow::{bail, Context, Result};

/// Names of the subdirectories of `dir`, sorted.
pub fn subdirectories(dir: &Path) -> Result<Vec<String>> {
    let mut names: Vec<String> = std::fs::read_dir(dir)
        .with_context(|| format!("cannot list {}", dir.display()))?
        .flatten()
        .filter(|e| e.path().is_dir())
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    Ok(names)
}

/// Locates the share directories of ROS packages.
pub type ShareResolver = Box<dyn Fn(&str) -> Result<PathBuf> + Send>;

/// Share directory of a package in the sourced workspace, via AMENT_PREFIX_PATH.
pub fn ament_share(package: &str) -> Result<PathBuf> {
    let prefixes = std::env::var_os("AMENT_PREFIX_PATH").unwrap_or_default();
    for prefix in std::env::split_paths(&prefixes) {
        let dir = prefix.join("share").join(package);
        if dir.is_dir() {
            return Ok(dir);
        }
    }
    bail!("package {package} not found in AMENT_PREFIX_PATH; is the workspace built and sourced?")
}
