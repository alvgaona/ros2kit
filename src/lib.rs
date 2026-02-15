//! ROS 2 toolkit library for workspace scanning, ament environment discovery,
//! process management, and build integration.

/// Ament environment discovery: installed packages, executables, launch files, interfaces.
pub mod ament;
/// Colcon build integration (requires `colcon` feature).
#[cfg(feature = "colcon")]
pub mod build;
/// Launch file argument parsing via Python helper.
pub mod launch;
/// ROS 2 log file discovery and tailing.
pub mod log;
/// Process launching, lifecycle management, and output streaming.
pub mod process;
/// Workspace scanning, package discovery, and overlay path management.
pub mod workspace;

pub use ament::{Env, Executable, InterfaceDef, InterfaceKind, LaunchFile, Package};
#[cfg(feature = "colcon")]
pub use build::{BuildConfig, BuildResult, BuildStatus, Builder, CmakeBuildType, PackageSelection};
pub use launch::LaunchArg;
pub use process::{LaunchType, Launcher};
pub use workspace::{
    PackageBuildStatus, PackageBuildType, Workspace, WorkspaceLayout, WorkspacePackage,
};
