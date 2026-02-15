use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::process::Stdio;

use anyhow::{Context, Result};
use tokio::process::{Child, Command};
use tokio::sync::mpsc;

fn log_dir() -> PathBuf {
    dirs::cache_dir()
        .unwrap_or_else(|| PathBuf::from("."))
        .join("rosx")
        .join("logs")
}

/// Returns the log file path for a given process name and PID.
pub fn log_path_for_name_with_pid(name: &str, pid: u32) -> PathBuf {
    let safe_name = name.replace('/', "_").trim_start_matches('_').to_string();
    log_dir().join(format!("{}_{}.log", safe_name, pid))
}

/// Creates the log directory if it does not already exist.
pub async fn ensure_log_dir() -> Result<()> {
    tokio::fs::create_dir_all(log_dir()).await?;
    Ok(())
}

/// Describes how a process was launched.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum LaunchType {
    /// A single ROS 2 node identified by package and executable name.
    Node { package: String, executable: String },
    /// A ROS 2 launch file identified by package and filename.
    LaunchFile { package: String, file: String },
}

/// Options for launching a single ROS 2 node.
pub struct NodeLaunchOptions<'a> {
    /// ROS 2 package name.
    pub package: &'a str,
    /// Executable name within the package.
    pub executable: &'a str,
    /// ROS parameter key-value pairs passed via `--ros-args -p`.
    pub params: &'a [(&'a str, &'a str)],
    /// ROS namespace to remap the node into.
    pub namespace: &'a str,
    /// Resolved filesystem path to the executable.
    pub exec_path: &'a Path,
    /// Additional environment variables for the spawned process.
    pub env: &'a [(&'a str, &'a str)],
}

/// Options for launching a ROS 2 launch file.
pub struct LaunchFileLaunchOptions<'a> {
    /// ROS 2 package name containing the launch file.
    pub package: &'a str,
    /// Launch filename (e.g. `my.launch.py`).
    pub file: &'a str,
    /// Resolved filesystem path to the launch file.
    pub path: &'a Path,
    /// Path to the Python interpreter used to run the launch file.
    pub python: &'a Path,
    /// Launch argument key-value pairs.
    pub args: &'a [(&'a str, &'a str)],
    /// Additional environment variables for the spawned process.
    pub env: &'a [(&'a str, &'a str)],
}

/// Result returned after successfully launching a process.
pub struct LaunchResult {
    /// OS process ID of the spawned process.
    pub pid: u32,
    /// Filesystem path to the log file capturing stdout/stderr.
    pub log_path: PathBuf,
    /// Channel receiver that streams log output lines in real time.
    pub output_rx: mpsc::Receiver<String>,
}

/// Manages spawning, tracking, and stopping ROS 2 processes.
pub struct Launcher {
    processes: HashMap<u32, ManagedProcess>,
}

struct ManagedProcess {
    child: Option<Child>,
}

impl Launcher {
    /// Creates a new launcher with no tracked processes.
    pub fn new() -> Self {
        Self {
            processes: HashMap::new(),
        }
    }

    /// Spawns a ROS 2 node process and begins tailing its log output.
    pub async fn launch_node(&mut self, opts: NodeLaunchOptions<'_>) -> Result<LaunchResult> {
        if !opts.exec_path.exists() {
            anyhow::bail!(
                "Executable not found: {}/{}. Package may not be built or installed.",
                opts.package,
                opts.executable,
            );
        }

        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let perms = opts
                .exec_path
                .metadata()
                .context("Failed to read executable metadata")?
                .permissions();
            if perms.mode() & 0o111 == 0 {
                anyhow::bail!("File is not executable: {}", opts.exec_path.display(),);
            }
        }

        let _ = std::fs::create_dir_all(log_dir());

        let timestamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_millis())
            .unwrap_or(0);
        let log_path = log_dir().join(format!("{}_{}.log", opts.executable, timestamp));

        let log_file = std::fs::File::create(&log_path).context("Failed to create log file")?;
        let log_file_err = log_file.try_clone().context("Failed to clone log file")?;

        let mut cmd = Command::new(opts.exec_path);

        let has_ros_args = !opts.params.is_empty() || !opts.namespace.is_empty();
        if has_ros_args {
            cmd.arg("--ros-args");
            for (name, value) in opts.params {
                cmd.arg("-p");
                cmd.arg(format!("{}:={}", name, value));
            }
            if !opts.namespace.is_empty() {
                let ns = if opts.namespace.starts_with('/') {
                    opts.namespace.to_string()
                } else {
                    format!("/{}", opts.namespace)
                };
                cmd.args(["-r", &format!("__ns:={}", ns)]);
            }
        }

        cmd.env("PYTHONUNBUFFERED", "1");
        for (key, value) in opts.env {
            cmd.env(key, value);
        }
        cmd.stdout(Stdio::from(log_file))
            .stderr(Stdio::from(log_file_err));
        #[cfg(unix)]
        cmd.process_group(0);
        let child = cmd
            .spawn()
            .with_context(|| format!("Failed to spawn {}", opts.exec_path.display()))?;

        let pid = child.id().unwrap_or(0);
        let rx = tail_log_file(&log_path, 0);

        self.processes
            .insert(pid, ManagedProcess { child: Some(child) });

        Ok(LaunchResult {
            pid,
            log_path,
            output_rx: rx,
        })
    }

    /// Spawns a ROS 2 launch file via the Python helper and pipes its output to a log file.
    pub async fn launch_file(&mut self, opts: LaunchFileLaunchOptions<'_>) -> Result<LaunchResult> {
        let name = opts
            .file
            .trim_end_matches(".launch.py")
            .trim_end_matches(".launch.xml")
            .trim_end_matches(".launch.yaml")
            .to_string();

        let _ = ensure_log_dir().await;

        let script = crate::launch::helper_path()?;
        let mut cmd = Command::new(opts.python);
        cmd.arg(&script);
        cmd.arg("run");
        cmd.arg(opts.path);
        for (arg_name, arg_value) in opts.args {
            if !arg_value.is_empty() {
                cmd.arg(format!("{}:={}", arg_name, arg_value));
            }
        }
        cmd.env("PYTHONUNBUFFERED", "1");
        for (key, value) in opts.env {
            cmd.env(key, value);
        }
        cmd.stdout(Stdio::piped()).stderr(Stdio::piped());
        #[cfg(unix)]
        cmd.process_group(0);
        let mut child = cmd.spawn().context("Failed to spawn launch file")?;

        let pid = child.id().unwrap_or(0);
        let log_path = log_path_for_name_with_pid(&name, pid);

        let stdout = child.stdout.take();
        let stderr = child.stderr.take();
        let rx = pipe_to_log_file(stdout, stderr, &log_path);

        self.processes
            .insert(pid, ManagedProcess { child: Some(child) });

        Ok(LaunchResult {
            pid,
            log_path,
            output_rx: rx,
        })
    }

    /// Stops a tracked process by PID, sending SIGTERM then SIGKILL to its process group.
    pub async fn stop(&mut self, pid: u32) -> Result<()> {
        if let Some(process) = self.processes.remove(&pid) {
            #[cfg(unix)]
            {
                let pgid = format!("-{}", pid);
                let _ = tokio::process::Command::new("kill")
                    .args(["-TERM", &pgid])
                    .output()
                    .await;

                tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

                let _ = tokio::process::Command::new("kill")
                    .args(["-KILL", &pgid])
                    .output()
                    .await;
            }

            if let Some(mut child) = process.child {
                let _ = child.wait().await;
            }
        }
        Ok(())
    }

    /// Re-tracks an externally running process and tails its existing log file from the current end.
    pub fn restore(&mut self, pid: u32, log_path: &Path) -> mpsc::Receiver<String> {
        let start_pos = std::fs::metadata(log_path).map(|m| m.len()).unwrap_or(0);
        let rx = tail_log_file(log_path, start_pos);
        self.processes.insert(pid, ManagedProcess { child: None });
        rx
    }

    /// Re-tracks a process by PID, discovering its ROS 2 log file automatically.
    pub fn restore_by_pid(&mut self, pid: u32) -> mpsc::Receiver<String> {
        let rx = tail_ros2_log(pid);
        self.processes.insert(pid, ManagedProcess { child: None });
        rx
    }

    /// Removes processes that have exited and returns their PIDs.
    pub fn cleanup_exited(&mut self) -> Vec<u32> {
        let mut exited = Vec::new();
        for (&id, process) in &mut self.processes {
            if let Some(ref mut child) = process.child {
                if let Ok(Some(_)) = child.try_wait() {
                    exited.push(id);
                }
            } else {
                if !is_process_alive(id) {
                    exited.push(id);
                }
            }
        }
        for id in &exited {
            self.processes.remove(id);
        }
        exited
    }
}

impl Default for Launcher {
    fn default() -> Self {
        Self::new()
    }
}

/// Checks whether a process with the given PID is still running.
pub fn is_process_alive(pid: u32) -> bool {
    #[cfg(unix)]
    {
        std::process::Command::new("kill")
            .args(["-0", &pid.to_string()])
            .output()
            .map(|o| o.status.success())
            .unwrap_or(false)
    }
    #[cfg(not(unix))]
    {
        false
    }
}

fn pipe_to_log_file(
    stdout: Option<tokio::process::ChildStdout>,
    stderr: Option<tokio::process::ChildStderr>,
    log_path: &Path,
) -> mpsc::Receiver<String> {
    use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};

    let (tx, rx) = mpsc::channel(1000);

    if let Some(stdout) = stdout {
        let tx = tx.clone();
        let log_path = log_path.to_path_buf();
        tokio::spawn(async move {
            let mut log_file = tokio::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&log_path)
                .await
                .ok();

            let mut reader = BufReader::new(stdout).lines();
            while let Ok(Some(line)) = reader.next_line().await {
                if let Some(ref mut f) = log_file {
                    let _ = f.write_all(line.as_bytes()).await;
                    let _ = f.write_all(b"\n").await;
                    let _ = f.flush().await;
                }
                let _ = tx.send(line).await;
            }
        });
    }

    if let Some(stderr) = stderr {
        let log_path = log_path.to_path_buf();
        tokio::spawn(async move {
            let mut log_file = tokio::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&log_path)
                .await
                .ok();

            let mut reader = BufReader::new(stderr).lines();
            while let Ok(Some(line)) = reader.next_line().await {
                if let Some(ref mut f) = log_file {
                    let _ = f.write_all(line.as_bytes()).await;
                    let _ = f.write_all(b"\n").await;
                    let _ = f.flush().await;
                }
                let _ = tx.send(line).await;
            }
        });
    }

    rx
}

fn tail_log_file(log_path: &Path, start_pos: u64) -> mpsc::Receiver<String> {
    let (tx, rx) = mpsc::channel(1000);
    let log_path = log_path.to_path_buf();

    std::thread::spawn(move || {
        use std::io::{Read, Seek, SeekFrom};

        for _ in 0..50 {
            std::thread::sleep(std::time::Duration::from_millis(100));
            if log_path.exists() {
                break;
            }
        }

        let mut file = match std::fs::File::open(&log_path) {
            Ok(f) => f,
            Err(_) => return,
        };

        let mut pos: u64 = start_pos;
        let mut buffer = vec![0u8; 4096];
        let mut partial_line = String::new();

        loop {
            let file_len = file.metadata().map(|m| m.len()).unwrap_or(0);

            if file_len > pos {
                if file.seek(SeekFrom::Start(pos)).is_err() {
                    break;
                }

                let to_read = (file_len - pos) as usize;
                let read_size = to_read.min(buffer.len());

                match file.read(&mut buffer[..read_size]) {
                    Ok(0) => {}
                    Ok(n) => {
                        pos += n as u64;
                        let chunk = String::from_utf8_lossy(&buffer[..n]);
                        let combined = format!("{}{}", partial_line, chunk);
                        partial_line.clear();

                        let mut lines: Vec<&str> = combined.split('\n').collect();
                        if !combined.ends_with('\n') && !lines.is_empty() {
                            partial_line = lines.pop().unwrap_or("").to_string();
                        }

                        for line in lines {
                            let trimmed = line.trim();
                            if !trimmed.is_empty() && tx.blocking_send(trimmed.to_string()).is_err()
                            {
                                return;
                            }
                        }
                    }
                    Err(_) => break,
                }
            } else {
                std::thread::sleep(std::time::Duration::from_millis(100));
            }
        }
    });

    rx
}

fn tail_ros2_log(pid: u32) -> mpsc::Receiver<String> {
    let (tx, rx) = mpsc::channel(1000);

    std::thread::spawn(move || {
        use std::io::{Read, Seek, SeekFrom};

        let mut log_path = None;
        for _ in 0..100 {
            std::thread::sleep(std::time::Duration::from_millis(100));
            if let Some(path) = crate::log::find_log_by_pid(pid) {
                log_path = Some(path);
                break;
            }
        }

        let log_path = match log_path {
            Some(p) => p,
            None => return,
        };

        let mut file = match std::fs::File::open(&log_path) {
            Ok(f) => f,
            Err(_) => return,
        };

        let mut pos: u64 = 0;
        let mut buffer = vec![0u8; 4096];
        let mut partial_line = String::new();

        loop {
            let file_len = file.metadata().map(|m| m.len()).unwrap_or(0);

            if file_len > pos {
                if file.seek(SeekFrom::Start(pos)).is_err() {
                    break;
                }

                let to_read = (file_len - pos) as usize;
                let read_size = to_read.min(buffer.len());

                match file.read(&mut buffer[..read_size]) {
                    Ok(0) => {}
                    Ok(n) => {
                        pos += n as u64;
                        let chunk = String::from_utf8_lossy(&buffer[..n]);
                        let combined = format!("{}{}", partial_line, chunk);
                        partial_line.clear();

                        let mut lines: Vec<&str> = combined.split('\n').collect();
                        if !combined.ends_with('\n') && !lines.is_empty() {
                            partial_line = lines.pop().unwrap_or("").to_string();
                        }

                        for line in lines {
                            let trimmed = line.trim();
                            if !trimmed.is_empty() && tx.blocking_send(trimmed.to_string()).is_err()
                            {
                                return;
                            }
                        }
                    }
                    Err(_) => break,
                }
            } else {
                std::thread::sleep(std::time::Duration::from_millis(100));
            }
        }
    });

    rx
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_is_process_alive_nonexistent() {
        assert!(!is_process_alive(999_999_999));
    }
}
