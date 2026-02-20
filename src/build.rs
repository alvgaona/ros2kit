use std::path::PathBuf;
use std::process::Stdio;
use std::time::Duration;

use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};

use anyhow::{Context, Result};
use tokio::process::Command;
use tokio::sync::{mpsc, watch};

/// Configuration for a colcon build invocation.
#[derive(Debug, Clone)]
pub struct BuildConfig {
    /// Root path of the ROS 2 workspace.
    pub workspace_root: PathBuf,
    /// Which packages to include in the build.
    pub packages: PackageSelection,
    /// Extra arguments forwarded to CMake.
    pub cmake_args: Vec<String>,
    /// CMake build type (Debug, Release, etc.).
    pub build_type: CmakeBuildType,
    /// Maximum number of parallel colcon workers, if limited.
    pub parallel_jobs: Option<usize>,
}

/// Selects which packages to build.
#[derive(Debug, Clone)]
pub enum PackageSelection {
    /// Build every package in the workspace.
    All,
    /// Build only the listed packages.
    Packages(Vec<String>),
    /// Build a package and all of its dependencies.
    UpTo(String),
}

/// CMake build type passed via `-DCMAKE_BUILD_TYPE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CmakeBuildType {
    /// Unoptimized build with full debug symbols.
    Debug,
    /// Fully optimized build with no debug symbols.
    Release,
    /// Optimized build that retains debug symbols.
    RelWithDebInfo,
}

impl CmakeBuildType {
    fn as_str(&self) -> &str {
        match self {
            CmakeBuildType::Debug => "Debug",
            CmakeBuildType::Release => "Release",
            CmakeBuildType::RelWithDebInfo => "RelWithDebInfo",
        }
    }
}

/// Current state of a running or completed build.
#[derive(Debug, Clone)]
pub enum BuildStatus {
    /// A package is currently being compiled.
    Building { package: String, progress: String },
    /// The build process has exited.
    Finished { success: bool, duration: Duration },
    /// A specific package failed to build.
    Failed { package: String, error: String },
}

/// Handles returned after starting a build, used to stream output and status.
pub struct BuildResult {
    /// Receiver for raw stdout/stderr lines from colcon.
    pub output_rx: mpsc::UnboundedReceiver<String>,
    /// Receiver for the latest parsed build status.
    pub status_rx: watch::Receiver<BuildStatus>,
}

/// Manages a single colcon build subprocess.
pub struct Builder {
    pgid: Arc<AtomicU32>,
}

impl Builder {
    pub fn new() -> Self {
        Self {
            pgid: Arc::new(AtomicU32::new(0)),
        }
    }

    pub fn is_building(&self) -> bool {
        self.pgid.load(Ordering::Relaxed) != 0
    }

    pub async fn build(&mut self, config: BuildConfig) -> Result<BuildResult> {
        if self.is_building() {
            anyhow::bail!("Build already in progress. Cancel first.");
        }

        let mut cmd = Command::new("colcon");
        cmd.arg("build");
        cmd.arg("--build-base")
            .arg(config.workspace_root.join("build"));
        cmd.arg("--install-base")
            .arg(config.workspace_root.join("install"));
        cmd.arg("--base-paths").arg(&config.workspace_root);

        match &config.packages {
            PackageSelection::All => {}
            PackageSelection::Packages(pkgs) => {
                cmd.arg("--packages-select");
                for pkg in pkgs {
                    cmd.arg(pkg);
                }
            }
            PackageSelection::UpTo(pkg) => {
                cmd.arg("--packages-up-to").arg(pkg);
            }
        }

        cmd.arg("--cmake-args")
            .arg(format!("-DCMAKE_BUILD_TYPE={}", config.build_type.as_str()));
        for arg in &config.cmake_args {
            cmd.arg(arg);
        }

        if let Some(jobs) = config.parallel_jobs {
            cmd.arg("--parallel-workers").arg(jobs.to_string());
        }

        cmd.arg("--event-handlers").arg("console_cohesion+");

        cmd.current_dir(&config.workspace_root);
        cmd.stdout(Stdio::piped());
        cmd.stderr(Stdio::piped());
        #[cfg(unix)]
        cmd.process_group(0);

        let mut child = cmd
            .spawn()
            .context("Failed to spawn colcon build. Is colcon installed?")?;

        if let Some(pid) = child.id() {
            self.pgid.store(pid, Ordering::Relaxed);
        }

        let stdout = child.stdout.take();
        let stderr = child.stderr.take();

        let (output_tx, output_rx) = mpsc::unbounded_channel();
        let (status_tx, status_rx) = watch::channel(BuildStatus::Building {
            package: String::new(),
            progress: "Starting...".to_string(),
        });

        let start_time = std::time::Instant::now();

        if let Some(stdout) = stdout {
            let tx = output_tx.clone();
            let status = status_tx.clone();
            tokio::spawn(async move {
                use tokio::io::{AsyncBufReadExt, BufReader};
                let mut reader = BufReader::new(stdout).lines();
                while let Ok(Some(line)) = reader.next_line().await {
                    parse_and_update_status(&line, &status);
                    let _ = tx.send(line);
                }
            });
        }

        if let Some(stderr) = stderr {
            let tx = output_tx.clone();
            tokio::spawn(async move {
                use tokio::io::{AsyncBufReadExt, BufReader};
                let mut reader = BufReader::new(stderr).lines();
                while let Ok(Some(line)) = reader.next_line().await {
                    let _ = tx.send(line);
                }
            });
        }

        let mut child_for_wait = child;
        let wait_status_tx = status_tx;
        let pgid_handle = self.pgid.clone();
        tokio::spawn(async move {
            let result = child_for_wait.wait().await;
            let duration = start_time.elapsed();
            let success = result.map(|s| s.success()).unwrap_or(false);
            pgid_handle.store(0, Ordering::Relaxed);
            let _ = wait_status_tx.send(BuildStatus::Finished { success, duration });
        });

        Ok(BuildResult {
            output_rx,
            status_rx,
        })
    }

    pub async fn cancel(&mut self) -> Result<()> {
        let pid = self.pgid.swap(0, Ordering::Relaxed);
        if pid == 0 {
            return Ok(());
        }

        #[cfg(unix)]
        {
            let pgid = format!("-{}", pid);
            let _ = tokio::process::Command::new("kill")
                .args(["-TERM", &pgid])
                .output()
                .await;

            tokio::time::sleep(Duration::from_secs(2)).await;

            let _ = tokio::process::Command::new("kill")
                .args(["-KILL", &pgid])
                .output()
                .await;
        }

        Ok(())
    }
}

impl Default for Builder {
    fn default() -> Self {
        Self::new()
    }
}

pub fn clean(workspace_root: &std::path::Path, packages: &[String]) -> Result<usize> {
    let mut removed = 0;
    for pkg in packages {
        for dir in &["build", "install", "log"] {
            let path = workspace_root.join(dir).join(pkg);
            if path.exists() {
                std::fs::remove_dir_all(&path)?;
                removed += 1;
            }
        }
    }
    Ok(removed)
}

fn parse_and_update_status(line: &str, status_tx: &watch::Sender<BuildStatus>) {
    if let Some(pkg) = line.strip_prefix("Starting >>> ") {
        let _ = status_tx.send(BuildStatus::Building {
            package: pkg.trim().to_string(),
            progress: "Building".to_string(),
        });
    } else if line.starts_with("Failed   <<< ") {
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() >= 3 {
            let _ = status_tx.send(BuildStatus::Failed {
                package: parts[2].to_string(),
                error: line.to_string(),
            });
        }
    }
}
