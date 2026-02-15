use std::path::{Path, PathBuf};
use std::process::Stdio;

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use tokio::process::Command;

const HELPER_SCRIPT: &str = include_str!("../assets/helper.py");

/// A declared argument from a ROS 2 launch file.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LaunchArg {
    /// Argument name as declared in the launch file.
    pub name: String,
    /// Default value for the argument, or empty if none is specified.
    pub default_value: String,
}

fn helper_script_path() -> PathBuf {
    dirs::cache_dir()
        .unwrap_or_else(|| PathBuf::from("."))
        .join("ros2kit")
        .join("launch_helper.py")
}

fn ensure_helper_script() -> Result<PathBuf> {
    let path = helper_script_path();
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(&path, HELPER_SCRIPT)?;
    Ok(path)
}

/// Writes the embedded Python helper script to the cache directory and returns its path.
pub fn helper_path() -> Result<PathBuf> {
    ensure_helper_script()
}

#[derive(Deserialize)]
struct RawArg {
    name: String,
    default: String,
}

/// Invokes the helper script to extract declared arguments from a ROS 2 launch file.
pub async fn parse_launch_args(python: &Path, launch_file_path: &Path) -> Result<Vec<LaunchArg>> {
    let script = ensure_helper_script()?;

    let output = Command::new(python)
        .args([
            script.as_os_str(),
            "show-args".as_ref(),
            launch_file_path.as_os_str(),
        ])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .await
        .context("Failed to run launch helper")?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        anyhow::bail!("Launch helper failed: {stderr}");
    }

    let stdout = String::from_utf8_lossy(&output.stdout);
    let raw_args: Vec<RawArg> =
        serde_json::from_str(&stdout).context("Failed to parse launch args JSON")?;

    Ok(raw_args
        .into_iter()
        .map(|a| LaunchArg {
            name: a.name,
            default_value: a.default,
        })
        .collect())
}
