use std::collections::HashSet;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use ignore::WalkBuilder;

/// Layout type of a ROS 2 workspace.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WorkspaceLayout {
    /// Standard colcon layout with `src/` containing packages.
    Standard,
    /// Pixi-managed workspace with `pixi.toml` at root. Packages may be anywhere.
    Pixi,
}

/// A ROS 2 workspace directory containing source packages and optionally build artifacts.
#[derive(Debug, Clone)]
pub struct Workspace {
    /// Root directory of the workspace.
    pub root: PathBuf,
}

impl Workspace {
    /// Returns the workspace display name (last path component, or `"."` for cwd).
    pub fn name(&self) -> &str {
        self.root
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or(".")
    }

    /// Detects the workspace layout by checking for `pixi.toml` or `pixi.lock`.
    pub fn layout(&self) -> WorkspaceLayout {
        if self.root.join("pixi.toml").is_file() || self.root.join("pixi.lock").is_file() {
            WorkspaceLayout::Pixi
        } else {
            WorkspaceLayout::Standard
        }
    }

    pub fn is_workspace(&self) -> bool {
        self.root.join("src").is_dir()
            || self.root.join("install").is_dir()
            || self.root.join("pixi.toml").is_file()
    }

    /// Returns `true` if an `install/` directory exists in this workspace.
    pub fn has_install(&self) -> bool {
        self.root.join("install").is_dir()
    }

    /// Returns `true` if the workspace contains source packages.
    ///
    /// For standard workspaces, checks that `src/` exists and is non-empty.
    /// For pixi workspaces, scans for `package.xml` files anywhere under the root.
    pub fn has_source(&self) -> bool {
        match self.layout() {
            WorkspaceLayout::Standard => {
                let src = self.root.join("src");
                if !src.is_dir() {
                    return false;
                }
                if let Ok(mut entries) = std::fs::read_dir(&src) {
                    entries.any(|e| e.is_ok())
                } else {
                    false
                }
            }
            WorkspaceLayout::Pixi => self.package_count() > 0,
        }
    }

    /// Returns `true` if at least one package has been built and installed
    /// (i.e. has an ament index entry under `install/`).
    pub fn is_built(&self) -> bool {
        let install_dir = self.root.join("install");
        if !install_dir.is_dir() {
            return false;
        }
        let Ok(entries) = std::fs::read_dir(&install_dir) else {
            return false;
        };
        entries.flatten().any(|e| {
            let path = e.path();
            path.is_dir()
                && path
                    .join("share")
                    .join("ament_index")
                    .join("resource_index")
                    .join("packages")
                    .is_dir()
        })
    }

    /// Returns `true` if colcon build artifacts exist (`log/` or `build/` directories).
    pub fn has_logs(&self) -> bool {
        self.root.join("log").is_dir() || self.root.join("build").is_dir()
    }

    /// Returns the number of source packages found by scanning for `package.xml` files.
    pub fn package_count(&self) -> usize {
        scan(&self.root).len()
    }

    /// Returns the number of installed packages with an ament index under `install/`.
    pub fn install_package_count(&self) -> usize {
        let install_dir = self.root.join("install");
        if !install_dir.is_dir() {
            return 0;
        }
        let Ok(entries) = std::fs::read_dir(&install_dir) else {
            return 0;
        };
        entries
            .flatten()
            .filter(|e| {
                let path = e.path();
                path.is_dir()
                    && path
                        .join("share")
                        .join("ament_index")
                        .join("resource_index")
                        .join("packages")
                        .is_dir()
            })
            .count()
    }

    /// Returns canonicalized ament prefix paths from `install/` subdirectories.
    pub fn install_prefixes(&self) -> Vec<PathBuf> {
        self.overlay_paths().ament_prefixes
    }

    /// Collects all environment overlay paths from the workspace's `install/` directory.
    ///
    /// Returns ament prefixes, Python site-packages paths, and library paths
    /// needed to overlay this workspace's installed packages onto the environment.
    pub fn overlay_paths(&self) -> OverlayPaths {
        let install_dir = self.root.join("install");
        if !install_dir.is_dir() {
            return OverlayPaths::default();
        }

        let canonical = install_dir
            .canonicalize()
            .unwrap_or_else(|_| install_dir.clone());

        let mut paths = OverlayPaths::default();

        let Ok(entries) = std::fs::read_dir(&canonical) else {
            return paths;
        };

        for entry in entries.flatten() {
            let sub = entry.path();
            if !sub.is_dir() {
                continue;
            }
            if !sub
                .join("share")
                .join("ament_index")
                .join("resource_index")
                .join("packages")
                .is_dir()
            {
                continue;
            }
            collect_prefix_paths(&sub, &mut paths);
        }

        if paths.ament_prefixes.is_empty() {
            collect_prefix_paths(&canonical, &mut paths);
        }

        paths
    }
}

/// Environment paths collected from a workspace's `install/` directory for overlay.
#[derive(Debug, Clone, Default)]
pub struct OverlayPaths {
    /// Ament prefix paths (`AMENT_PREFIX_PATH` entries).
    pub ament_prefixes: Vec<PathBuf>,
    /// Python site-packages paths (`PYTHONPATH` entries).
    pub python_paths: Vec<PathBuf>,
    /// Library paths (`LD_LIBRARY_PATH` / `DYLD_LIBRARY_PATH` entries).
    pub lib_paths: Vec<PathBuf>,
}

fn collect_prefix_paths(prefix: &Path, paths: &mut OverlayPaths) {
    paths.ament_prefixes.push(prefix.to_path_buf());
    let lib_dir = prefix.join("lib");
    if !lib_dir.is_dir() {
        return;
    }
    paths.lib_paths.push(lib_dir.clone());
    for py_dir in &["python3.12", "python3.11", "python3.10", "python3.9"] {
        let site_packages = lib_dir.join(py_dir).join("site-packages");
        if site_packages.is_dir() {
            paths.python_paths.push(site_packages.clone());
            if let Ok(entries) = std::fs::read_dir(&site_packages) {
                for entry in entries.flatten() {
                    if entry.file_name().to_string_lossy().ends_with(".egg-link") {
                        if let Ok(content) = std::fs::read_to_string(entry.path()) {
                            if let Some(line) = content.lines().next() {
                                let target = PathBuf::from(line.trim());
                                if target.is_dir() {
                                    paths.python_paths.push(target);
                                }
                            }
                        }
                    }
                }
            }
            break;
        }
    }
}

/// Build system type declared in a package's `package.xml` under `<export><build_type>`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PackageBuildType {
    AmentCmake,
    AmentPython,
    Cmake,
    AmentCargo,
    Unknown,
}

/// A ROS 2 source package discovered by scanning a workspace for `package.xml` files.
#[derive(Debug, Clone)]
pub struct WorkspacePackage {
    /// Package name from `<name>` in `package.xml`.
    pub name: String,
    /// Package version from `<version>` in `package.xml`.
    pub version: String,
    /// Package description from `<description>` in `package.xml`.
    pub description: String,
    /// Path to the package directory (parent of `package.xml`).
    pub path: PathBuf,
    /// The workspace this package belongs to.
    pub workspace: Arc<Workspace>,
    /// Build system type from `<export><build_type>` in `package.xml`.
    pub build_type: PackageBuildType,
    /// Deduplicated, sorted list of all dependency package names from `package.xml`.
    pub dependencies: Vec<String>,
}

/// Build state of a workspace package relative to its install directory.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PackageBuildStatus {
    /// No ament index entry exists in `install/`.
    NotBuilt,
    /// Installed and up-to-date (install marker is newer than all source files).
    Built,
    /// Source files have been modified since the last build.
    Dirty,
}

impl WorkspacePackage {
    /// Returns the path to this package's `package.xml`.
    pub fn package_xml(&self) -> PathBuf {
        self.path.join("package.xml")
    }

    /// Returns the path to `CMakeLists.txt` if it exists in the package directory.
    pub fn cmake_lists(&self) -> Option<PathBuf> {
        let path = self.path.join("CMakeLists.txt");
        if path.is_file() { Some(path) } else { None }
    }

    /// Returns the path to `setup.py` if it exists in the package directory.
    pub fn setup_py(&self) -> Option<PathBuf> {
        let path = self.path.join("setup.py");
        if path.is_file() { Some(path) } else { None }
    }

    /// Returns the path to `Cargo.toml` if it exists in the package directory.
    pub fn cargo_toml(&self) -> Option<PathBuf> {
        let path = self.path.join("Cargo.toml");
        if path.is_file() { Some(path) } else { None }
    }

    pub fn latest_build_log(&self) -> Option<String> {
        let log_file = self
            .workspace
            .root
            .join("log")
            .join("latest_build")
            .join(&self.name)
            .join("stdout_stderr.log");
        std::fs::read_to_string(&log_file).ok()
    }

    pub fn build_status(&self) -> PackageBuildStatus {
        let build_marker = self
            .workspace
            .root
            .join("build")
            .join(&self.name)
            .join("colcon_build.rc");

        if !build_marker.is_file() {
            return PackageBuildStatus::NotBuilt;
        }

        let rc = std::fs::read_to_string(&build_marker).unwrap_or_default();
        if rc.trim() != "0" {
            return PackageBuildStatus::Dirty;
        }

        let build_mtime = build_marker.metadata().and_then(|m| m.modified()).ok();

        let source_mtime = newest_source_mtime(&self.path);

        match (source_mtime, build_mtime) {
            (Some(src), Some(built)) if src > built => PackageBuildStatus::Dirty,
            _ => PackageBuildStatus::Built,
        }
    }
}

fn newest_source_mtime(dir: &Path) -> Option<std::time::SystemTime> {
    let mut newest = None;

    let walker = WalkBuilder::new(dir)
        .hidden(true)
        .git_ignore(true)
        .git_global(false)
        .git_exclude(true)
        .build();

    for entry in walker.flatten() {
        if !entry.file_type().is_some_and(|ft| ft.is_file()) {
            continue;
        }
        if let Ok(meta) = entry.path().metadata()
            && let Ok(mtime) = meta.modified()
        {
            newest = Some(match newest {
                Some(current) if mtime > current => mtime,
                Some(current) => current,
                None => mtime,
            });
        }
    }

    newest
}

/// Scans a single directory for ROS 2 source packages. Delegates to [`scan_multiple`].
pub fn scan(root: &Path) -> Vec<WorkspacePackage> {
    scan_multiple(&[root.to_path_buf()])
}

/// Scans multiple workspace directories for ROS 2 source packages in parallel.
///
/// Uses a single `ignore` walker across all roots. Duplicate roots (by canonical path)
/// are skipped. Results are sorted alphabetically by package name.
pub fn scan_multiple(roots: &[PathBuf]) -> Vec<WorkspacePackage> {
    if roots.is_empty() {
        return Vec::new();
    }

    let mut seen = HashSet::new();
    let workspaces: Vec<Arc<Workspace>> = roots
        .iter()
        .filter(|r| {
            let canonical = r.canonicalize().unwrap_or_else(|_| r.to_path_buf());
            seen.insert(canonical)
        })
        .map(|r| Arc::new(Workspace { root: r.clone() }))
        .collect();

    if workspaces.is_empty() {
        return Vec::new();
    }

    let packages = Mutex::new(Vec::new());

    let mut builder = WalkBuilder::new(&workspaces[0].root);
    for ws in &workspaces[1..] {
        builder.add(&ws.root);
    }

    builder
        .hidden(true)
        .follow_links(true)
        .git_ignore(true)
        .git_global(false)
        .git_exclude(true)
        .filter_entry(|entry| {
            if !entry.file_type().is_some_and(|ft| ft.is_dir()) {
                return true;
            }
            let name = entry.file_name().to_string_lossy();
            !matches!(
                name.as_ref(),
                "target" | "build" | "install" | "log" | "node_modules"
            )
        })
        .build_parallel()
        .run(|| {
            Box::new(|entry| {
                let entry = match entry {
                    Ok(e) => e,
                    Err(_) => return ignore::WalkState::Continue,
                };

                if !entry.file_type().is_some_and(|ft| ft.is_file()) {
                    return ignore::WalkState::Continue;
                }

                if entry.file_name() != "package.xml" {
                    return ignore::WalkState::Continue;
                }

                let path = entry.path();
                if let Some(dir) = path.parent()
                    && let Some(info) = parse_package_xml(path)
                {
                    let workspace = workspaces
                        .iter()
                        .find(|ws| path.starts_with(&ws.root))
                        .cloned()
                        .unwrap_or_else(|| {
                            Arc::new(Workspace {
                                root: dir.to_path_buf(),
                            })
                        });
                    packages.lock().unwrap().push(WorkspacePackage {
                        name: info.name,
                        version: info.version,
                        description: info.description,
                        path: dir.to_path_buf(),
                        workspace,
                        build_type: info.build_type,
                        dependencies: info.dependencies,
                    });
                }

                ignore::WalkState::Continue
            })
        });

    let mut packages = packages.into_inner().unwrap();
    packages.sort_by(|a, b| a.name.cmp(&b.name));
    packages
}

struct PackageXmlInfo {
    name: String,
    version: String,
    description: String,
    build_type: PackageBuildType,
    dependencies: Vec<String>,
}

const DEP_TAGS: &[&str] = &[
    "depend",
    "build_depend",
    "build_export_depend",
    "exec_depend",
    "buildtool_depend",
    "test_depend",
];

fn parse_package_xml(package_xml: &Path) -> Option<PackageXmlInfo> {
    use quick_xml::events::Event;
    use quick_xml::reader::Reader;

    let content = std::fs::read_to_string(package_xml).ok()?;
    let mut reader = Reader::from_str(&content);

    let mut name = None;
    let mut version = None;
    let mut description = None;
    let mut build_type = None;
    let mut dependencies = Vec::new();
    let mut seen_deps = HashSet::new();
    let mut in_export = false;
    let mut current_tag = String::new();

    loop {
        match reader.read_event() {
            Ok(Event::Start(e)) => {
                let tag = String::from_utf8_lossy(e.name().as_ref()).to_string();
                if tag == "export" {
                    in_export = true;
                }
                current_tag = tag;
            }
            Ok(Event::End(e)) => {
                if e.name().as_ref() == b"export" {
                    in_export = false;
                }
                current_tag.clear();
            }
            Ok(Event::Text(e)) => {
                let text = e.unescape().ok();
                match current_tag.as_str() {
                    "name" if name.is_none() => {
                        name = text.map(|t| t.trim().to_string());
                    }
                    "version" if version.is_none() => {
                        version = text.map(|t| t.trim().to_string());
                    }
                    "description" if description.is_none() => {
                        description = text.map(|t| t.trim().to_string());
                    }
                    "build_type" if in_export => {
                        build_type = text.map(|t| match t.trim() {
                            "ament_cmake" => PackageBuildType::AmentCmake,
                            "ament_python" => PackageBuildType::AmentPython,
                            "cmake" => PackageBuildType::Cmake,
                            "ament_cargo" => PackageBuildType::AmentCargo,
                            _ => PackageBuildType::Unknown,
                        });
                    }
                    tag if DEP_TAGS.contains(&tag) => {
                        if let Some(dep) = text.map(|t| t.trim().to_string())
                            && !dep.is_empty()
                            && seen_deps.insert(dep.clone())
                        {
                            dependencies.push(dep);
                        }
                    }
                    _ => {}
                }
            }
            Ok(Event::Eof) => break,
            Err(_) => return None,
            _ => {}
        }
    }

    dependencies.sort();

    Some(PackageXmlInfo {
        name: name?,
        version: version.unwrap_or_default(),
        description: description.unwrap_or_default(),
        build_type: build_type.unwrap_or(PackageBuildType::Unknown),
        dependencies,
    })
}
