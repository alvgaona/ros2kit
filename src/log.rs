use std::path::{Path, PathBuf};

/// Returns the ROS 2 log directory, checking `ROS_LOG_DIR`, `ROS_HOME`, and `~/.ros/log`.
pub fn ros2_log_dir() -> Option<PathBuf> {
    if let Ok(log_dir) = std::env::var("ROS_LOG_DIR") {
        return Some(PathBuf::from(log_dir));
    }

    if let Ok(ros_home) = std::env::var("ROS_HOME") {
        return Some(PathBuf::from(ros_home).join("log"));
    }

    dirs::home_dir().map(|h| h.join(".ros").join("log"))
}

/// Locates a log file for the given node name by searching filenames, PID, and recent run directories.
pub fn find_log_by_node(node_name: &str) -> Option<PathBuf> {
    let log_dir = ros2_log_dir()?;
    if !log_dir.exists() {
        return None;
    }

    if let Some(path) = search_log_by_name(&log_dir, node_name) {
        return Some(path);
    }

    if let Some(pid) = find_node_pid(node_name) {
        if let Some(path) = find_log_by_pid(pid) {
            return Some(path);
        }
        if let Some(path) = search_python_log_by_pid(&log_dir, pid) {
            return Some(path);
        }
    }

    let safe_name = node_name.trim_start_matches('/').replace('/', "_");

    let mut run_dirs: Vec<_> = std::fs::read_dir(&log_dir)
        .ok()?
        .filter_map(|e| e.ok())
        .filter(|e| e.path().is_dir())
        .collect();

    run_dirs.sort_by_key(|e| std::cmp::Reverse(e.path()));

    for run_dir in run_dirs.iter().take(5) {
        let run_path = run_dir.path();
        if let Ok(entries) = std::fs::read_dir(&run_path) {
            for entry in entries.filter_map(|e| e.ok()) {
                let file_name = entry.file_name();
                let name_str = file_name.to_string_lossy();
                if name_str.contains(&safe_name) && name_str.ends_with(".log") {
                    return Some(entry.path());
                }
            }
        }
    }

    None
}

/// Finds a log file in the ROS 2 log directory whose filename contains the given PID.
pub fn find_log_by_pid(pid: u32) -> Option<PathBuf> {
    let log_dir = ros2_log_dir()?;
    if !log_dir.exists() {
        return None;
    }

    let pid_pattern = format!("_{}_", pid);

    if let Ok(entries) = std::fs::read_dir(&log_dir) {
        for entry in entries.filter_map(|e| e.ok()) {
            let path = entry.path();
            if !path.is_file() {
                continue;
            }
            let name = entry.file_name();
            let name_str = name.to_string_lossy();
            if name_str.contains(&pid_pattern) && name_str.ends_with(".log") {
                return Some(path);
            }
        }
    }

    None
}

/// Finds a log file for a currently running node, preferring PID-based lookup over name-based.
pub fn find_log_for_running_node(node_name: &str) -> Option<PathBuf> {
    if let Some(pid) = find_node_pid(node_name)
        && let Some(path) = find_log_by_pid(pid)
    {
        return Some(path);
    }
    find_log_by_node(node_name)
}

/// Returns the PID of a running ROS 2 node by searching process arguments with `pgrep`.
pub fn find_node_pid(node_name: &str) -> Option<u32> {
    let safe_name = node_name.trim_start_matches('/');

    let patterns = [
        format!("__node:={}", safe_name),
        format!("--ros-args.*__node:={}", safe_name),
        safe_name.to_string(),
    ];

    for pattern in &patterns {
        let output = std::process::Command::new("pgrep")
            .arg("-f")
            .arg(pattern)
            .output()
            .ok();

        if let Some(output) = output
            && output.status.success()
        {
            let stdout = String::from_utf8_lossy(&output.stdout);
            if let Some(line) = stdout.lines().next()
                && let Ok(pid) = line.trim().parse()
            {
                return Some(pid);
            }
        }
    }

    None
}

/// Reads the last `max_lines` lines from the file at `path`.
pub fn read_tail(path: &Path, max_lines: usize) -> Option<Vec<String>> {
    use std::io::{BufRead, BufReader};

    let file = std::fs::File::open(path).ok()?;
    let reader = BufReader::new(file);
    let all_lines: Vec<String> = reader.lines().map_while(Result::ok).collect();

    if all_lines.len() > max_lines {
        Some(all_lines[all_lines.len() - max_lines..].to_vec())
    } else {
        Some(all_lines)
    }
}

fn search_log_by_name(log_dir: &Path, node_name: &str) -> Option<PathBuf> {
    let safe_name = node_name.trim_start_matches('/').replace('/', "_");

    let mut log_files: Vec<_> = std::fs::read_dir(log_dir)
        .ok()?
        .filter_map(|e| e.ok())
        .filter(|e| {
            let path = e.path();
            if !path.is_file() {
                return false;
            }
            let name = e.file_name();
            let name_str = name.to_string_lossy();
            name_str.starts_with(&safe_name) && name_str.ends_with(".log")
        })
        .collect();

    if log_files.is_empty() {
        return None;
    }

    log_files.sort_by(|a, b| {
        let time_a = a.metadata().and_then(|m| m.modified()).ok();
        let time_b = b.metadata().and_then(|m| m.modified()).ok();
        time_b.cmp(&time_a)
    });

    Some(log_files[0].path())
}

fn search_python_log_by_pid(log_dir: &Path, pid: u32) -> Option<PathBuf> {
    let pid_pattern = format!("_{}_", pid);

    let mut matches: Vec<_> = std::fs::read_dir(log_dir)
        .ok()?
        .filter_map(|e| e.ok())
        .filter(|e| {
            let name = e.file_name().to_string_lossy().to_string();
            name.starts_with("python") && name.contains(&pid_pattern) && name.ends_with(".log")
        })
        .collect();

    if matches.is_empty() {
        return None;
    }

    matches.sort_by(|a, b| {
        let time_a = a.metadata().and_then(|m| m.modified()).ok();
        let time_b = b.metadata().and_then(|m| m.modified()).ok();
        time_b.cmp(&time_a)
    });

    Some(matches[0].path())
}
