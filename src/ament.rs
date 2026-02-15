use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::Result;

use crate::workspace::Workspace;

/// The ament environment, providing access to installed ROS 2 packages.
///
/// Discovers packages, executables, launch files, and interfaces from
/// `AMENT_PREFIX_PATH` prefixes and workspace `install/` directories.
pub struct Env {
    prefixes: Vec<PathBuf>,
}

/// An installed ROS 2 package discovered from the ament index.
#[derive(Debug, Clone)]
pub struct Package {
    pub name: String,
    /// The ament prefix directory containing this package.
    pub prefix: PathBuf,
}

/// An executable file installed by a ROS 2 package.
#[derive(Debug, Clone)]
pub struct Executable {
    pub package: String,
    pub name: String,
    pub language: Language,
    pub size_bytes: u64,
    pub modified: std::time::SystemTime,
}

/// Detected language of a ROS 2 executable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Language {
    Python,
    Cpp,
    Unknown,
}

/// A launch file installed by a ROS 2 package.
#[derive(Debug, Clone)]
pub struct LaunchFile {
    pub package: String,
    pub name: String,
}

/// A ROS 2 interface definition (message, service, or action).
#[derive(Debug, Clone)]
pub struct InterfaceDef {
    pub package: String,
    pub kind: InterfaceKind,
    pub name: String,
}

/// The kind of a ROS 2 interface definition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterfaceKind {
    Message,
    Service,
    Action,
}

impl InterfaceKind {
    /// Returns the subdirectory name for this interface kind (`msg`, `srv`, or `action`).
    pub fn dir_name(&self) -> &str {
        match self {
            InterfaceKind::Message => "msg",
            InterfaceKind::Service => "srv",
            InterfaceKind::Action => "action",
        }
    }

    /// Returns the file extension for this interface kind (`msg`, `srv`, or `action`).
    pub fn extension(&self) -> &str {
        match self {
            InterfaceKind::Message => "msg",
            InterfaceKind::Service => "srv",
            InterfaceKind::Action => "action",
        }
    }
}

impl Env {
    /// Creates an ament environment from `AMENT_PREFIX_PATH`, overlaying cwd's `install/`.
    pub fn from_env() -> Result<Self> {
        Self::from_env_with_workspaces(&[])
    }

    /// Creates an ament environment from `AMENT_PREFIX_PATH`, overlaying registered
    /// workspaces' `install/` directories (in order) and cwd last (highest priority).
    /// Duplicate workspace roots are skipped.
    pub fn from_env_with_workspaces(workspaces: &[Arc<Workspace>]) -> Result<Self> {
        let path = std::env::var("AMENT_PREFIX_PATH")
            .map_err(|_| anyhow::anyhow!("AMENT_PREFIX_PATH is not set"))?;
        let mut env = Self::from_path(&path);

        let cwd_canonical = std::env::current_dir()
            .ok()
            .and_then(|p| p.canonicalize().ok());
        let mut seen_roots = std::collections::HashSet::new();
        if let Some(ref cwd) = cwd_canonical {
            seen_roots.insert(cwd.clone());
        }

        for ws in workspaces {
            let canonical = ws.root.canonicalize().unwrap_or_else(|_| ws.root.clone());
            if seen_roots.insert(canonical) {
                env.add_workspace_prefixes(ws);
            }
        }

        env.add_local_install_prefixes();
        Ok(env)
    }

    /// Creates an ament environment from a colon-separated prefix path string.
    pub fn from_path(path: &str) -> Self {
        let prefixes = path
            .split(':')
            .filter(|s| !s.is_empty())
            .map(PathBuf::from)
            .collect();
        Self { prefixes }
    }

    /// Returns the path to `python3`, searching ament prefixes first then falling back to PATH.
    pub fn python(&self) -> PathBuf {
        for prefix in &self.prefixes {
            let candidate = prefix.join("bin").join("python3");
            if candidate.exists() {
                return candidate;
            }
        }
        PathBuf::from("python3")
    }

    /// Returns all installed packages discovered from ament index directories.
    /// Earlier prefixes take priority for duplicate package names.
    pub fn packages(&self) -> Vec<Package> {
        let mut packages = Vec::new();
        let mut seen = std::collections::HashSet::new();

        for prefix in &self.prefixes {
            let index_dir = prefix
                .join("share")
                .join("ament_index")
                .join("resource_index")
                .join("packages");

            if let Ok(entries) = std::fs::read_dir(&index_dir) {
                for entry in entries.flatten() {
                    if let Some(name) = entry.file_name().to_str()
                        && seen.insert(name.to_string())
                    {
                        packages.push(Package {
                            name: name.to_string(),
                            prefix: prefix.clone(),
                        });
                    }
                }
            }
        }

        packages.sort_by(|a, b| a.name.cmp(&b.name));
        packages
    }

    /// Returns the ament prefix path for a package, or `None` if not found.
    pub fn package_prefix(&self, name: &str) -> Option<PathBuf> {
        for prefix in &self.prefixes {
            let marker = prefix
                .join("share")
                .join("ament_index")
                .join("resource_index")
                .join("packages")
                .join(name);
            if marker.exists() {
                return Some(prefix.clone());
            }
        }
        None
    }

    /// Returns all executables installed by a package (from `<prefix>/lib/<package>/`).
    pub fn executables(&self, package: &str) -> Vec<Executable> {
        let Some(prefix) = self.package_prefix(package) else {
            return Vec::new();
        };

        let lib_dir = prefix.join("lib").join(package);
        let mut executables = Vec::new();

        if let Ok(entries) = std::fs::read_dir(&lib_dir) {
            for entry in entries.flatten() {
                let path = entry.path();
                if !path.is_file() {
                    continue;
                }
                if let Some(name) = path.file_name().and_then(|n| n.to_str())
                    && is_executable(&path)
                {
                    let language = detect_language(&path);
                    let metadata = path.metadata().ok();
                    let size_bytes = metadata.as_ref().map(|m| m.len()).unwrap_or(0);
                    let modified = metadata
                        .as_ref()
                        .and_then(|m| m.modified().ok())
                        .unwrap_or_else(std::time::SystemTime::now);
                    executables.push(Executable {
                        package: package.to_string(),
                        name: name.to_string(),
                        language,
                        size_bytes,
                        modified,
                    });
                }
            }
        }

        executables.sort_by(|a, b| a.name.cmp(&b.name));
        executables
    }

    /// Returns launch files for a package (from `<prefix>/share/<package>/launch/`).
    pub fn launch_files(&self, package: &str) -> Vec<LaunchFile> {
        let Some(prefix) = self.package_prefix(package) else {
            return Vec::new();
        };

        let share_dir = prefix.join("share").join(package);
        let launch_dir = share_dir.join("launch");
        let mut launch_files = Vec::new();

        for dir in [&launch_dir, &share_dir] {
            if let Ok(entries) = std::fs::read_dir(dir) {
                for entry in entries.flatten() {
                    let path = entry.path();
                    if let Some(name) = path.file_name().and_then(|n| n.to_str())
                        && is_launch_file(name)
                        && !launch_files.iter().any(|lf: &LaunchFile| lf.name == name)
                    {
                        launch_files.push(LaunchFile {
                            package: package.to_string(),
                            name: name.to_string(),
                        });
                    }
                }
            }
        }

        launch_files.sort_by(|a, b| a.name.cmp(&b.name));
        launch_files
    }

    /// Returns interface definitions (messages, services, actions) for a package.
    pub fn interfaces(&self, package: &str) -> Vec<InterfaceDef> {
        let Some(prefix) = self.package_prefix(package) else {
            return Vec::new();
        };

        let share_dir = prefix.join("share").join(package);
        let mut defs = Vec::new();

        for (kind, dir_name, ext) in [
            (InterfaceKind::Message, "msg", "msg"),
            (InterfaceKind::Service, "srv", "srv"),
            (InterfaceKind::Action, "action", "action"),
        ] {
            let dir = share_dir.join(dir_name);
            if let Ok(entries) = std::fs::read_dir(&dir) {
                for entry in entries.flatten() {
                    let path = entry.path();
                    if let Some(name) = path.file_name().and_then(|n| n.to_str())
                        && let Some(stem) = name.strip_suffix(&format!(".{}", ext))
                    {
                        defs.push(InterfaceDef {
                            package: package.to_string(),
                            kind,
                            name: stem.to_string(),
                        });
                    }
                }
            }
        }

        defs.sort_by(|a, b| a.name.cmp(&b.name));
        defs
    }

    /// Resolves the full path to a launch file, checking `launch/` then `share/` directories.
    pub fn launch_file_path(&self, package: &str, file: &str) -> Option<PathBuf> {
        let prefix = self.package_prefix(package)?;
        let share_dir = prefix.join("share").join(package);

        let launch_path = share_dir.join("launch").join(file);
        if launch_path.exists() {
            return Some(launch_path);
        }

        let share_path = share_dir.join(file);
        if share_path.exists() {
            return Some(share_path);
        }

        None
    }

    /// Resolves the full path to an interface definition file (`.msg`, `.srv`, or `.action`).
    pub fn interface_path(&self, package: &str, kind: &str, name: &str) -> Option<PathBuf> {
        let prefix = self.package_prefix(package)?;
        let ext = kind;
        let path = prefix
            .join("share")
            .join(package)
            .join(kind)
            .join(format!("{}.{}", name, ext));
        if path.exists() { Some(path) } else { None }
    }

    fn add_workspace_prefixes(&mut self, workspace: &Workspace) {
        for prefix in workspace.install_prefixes().into_iter().rev() {
            if !self.prefixes.contains(&prefix) {
                self.prefixes.insert(0, prefix);
            }
        }
    }

    fn add_local_install_prefixes(&mut self) {
        let cwd = Workspace {
            root: PathBuf::from("."),
        };
        self.add_workspace_prefixes(&cwd);
    }

    /// Sets environment variables to overlay cwd's `install/` onto the current process.
    pub fn apply_workspace_overlay(&self) {
        self.apply_workspace_overlays(&[]);
    }

    /// Sets `AMENT_PREFIX_PATH`, `PYTHONPATH`, and library path environment variables
    /// to overlay all registered workspaces and cwd onto the current process.
    ///
    /// # Safety
    /// Must be called before spawning threads that read environment variables.
    pub fn apply_workspace_overlays(&self, workspaces: &[Arc<Workspace>]) {
        let cwd = Workspace {
            root: PathBuf::from("."),
        };

        let cwd_canonical = std::env::current_dir()
            .ok()
            .and_then(|p| p.canonicalize().ok());
        let mut seen_roots = std::collections::HashSet::new();
        if let Some(ref cwd) = cwd_canonical {
            seen_roots.insert(cwd.clone());
        }

        let mut all_paths = crate::workspace::OverlayPaths::default();

        for ws in workspaces {
            let canonical = ws.root.canonicalize().unwrap_or_else(|_| ws.root.clone());
            if !seen_roots.insert(canonical) {
                continue;
            }
            let paths = ws.overlay_paths();
            all_paths.ament_prefixes.extend(paths.ament_prefixes);
            all_paths.python_paths.extend(paths.python_paths);
            all_paths.lib_paths.extend(paths.lib_paths);
        }

        let cwd_paths = cwd.overlay_paths();
        all_paths.ament_prefixes.extend(cwd_paths.ament_prefixes);
        all_paths.python_paths.extend(cwd_paths.python_paths);
        all_paths.lib_paths.extend(cwd_paths.lib_paths);

        if all_paths.ament_prefixes.is_empty() {
            return;
        }

        let to_strings = |paths: &[PathBuf]| -> Vec<String> {
            paths.iter().map(|p| p.display().to_string()).collect()
        };

        prepend_env_path("AMENT_PREFIX_PATH", &to_strings(&all_paths.ament_prefixes));
        if !all_paths.python_paths.is_empty() {
            prepend_env_path("PYTHONPATH", &to_strings(&all_paths.python_paths));
        }
        if !all_paths.lib_paths.is_empty() {
            #[cfg(target_os = "macos")]
            prepend_env_path("DYLD_LIBRARY_PATH", &to_strings(&all_paths.lib_paths));
            #[cfg(not(target_os = "macos"))]
            prepend_env_path("LD_LIBRARY_PATH", &to_strings(&all_paths.lib_paths));
        }
    }
}

fn prepend_env_path(var: &str, new_paths: &[String]) {
    let existing = std::env::var(var).unwrap_or_default();
    let existing_parts: Vec<&str> = existing.split(':').filter(|s| !s.is_empty()).collect();

    let mut combined = Vec::new();
    for path in new_paths {
        if !existing_parts.contains(&path.as_str()) {
            combined.push(path.as_str());
        }
    }
    combined.extend(existing_parts);

    // SAFETY: rosx is single-threaded at the point where this is called (during init,
    // before spawning background tasks), so no other threads are reading env vars.
    unsafe { std::env::set_var(var, combined.join(":")) };
}

fn is_launch_file(name: &str) -> bool {
    name.ends_with(".launch.py") || name.ends_with(".launch.xml") || name.ends_with(".launch.yaml")
}

#[cfg(unix)]
fn is_executable(path: &Path) -> bool {
    use std::os::unix::fs::PermissionsExt;
    path.metadata()
        .map(|m| m.permissions().mode() & 0o111 != 0)
        .unwrap_or(false)
}

#[cfg(not(unix))]
fn is_executable(_path: &Path) -> bool {
    true
}

fn detect_language(path: &Path) -> Language {
    use std::io::Read;

    if let Some(name) = path.file_name().and_then(|n| n.to_str())
        && name.ends_with(".py")
    {
        return Language::Python;
    }

    let mut file = match std::fs::File::open(path) {
        Ok(f) => f,
        Err(_) => return Language::Unknown,
    };

    let mut buffer = [0u8; 64];
    let bytes_read = file.read(&mut buffer).unwrap_or(0);

    if bytes_read == 0 {
        return Language::Unknown;
    }

    if bytes_read >= 4 && &buffer[0..4] == b"\x7fELF" {
        return Language::Cpp;
    }

    if bytes_read >= 4 && (&buffer[0..2] == b"MZ" || &buffer[0..4] == b"\xCA\xFE\xBA\xBE") {
        return Language::Cpp;
    }

    if bytes_read >= 2 && &buffer[0..2] == b"#!" {
        let shebang = String::from_utf8_lossy(&buffer[0..bytes_read]);
        if shebang.contains("python") {
            return Language::Python;
        }
        return Language::Unknown;
    }

    if path.metadata().map(|m| m.len()).unwrap_or(0) > 1024 {
        return Language::Cpp;
    }

    Language::Unknown
}
