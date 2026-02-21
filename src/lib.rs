//! ROS 2 toolkit library for ament environment discovery, process management,
//! launch file support, and log file discovery.
//!
//! Optional features: `workspace` for workspace scanning and overlay management,
//! `colcon` for build integration.

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
#[cfg(feature = "mcap")]
pub mod record;
/// Workspace scanning, package discovery, and overlay path management (requires `workspace` feature).
#[cfg(feature = "workspace")]
pub mod workspace;

pub use ament::{Env, Executable, InterfaceDef, InterfaceKind, LaunchFile, LaunchFormat, Package};
#[cfg(feature = "colcon")]
pub use build::{BuildConfig, BuildResult, BuildStatus, Builder, CmakeBuildType, PackageSelection};
pub use launch::LaunchArg;
pub use process::{LaunchType, Launcher};
#[cfg(feature = "mcap")]
pub use record::{RecordConfig, RecordResult, RecordStats, Recorder, TopicRecording};
#[cfg(feature = "workspace")]
pub use workspace::{
    PackageBuildStatus, PackageBuildType, Workspace, WorkspaceLayout, WorkspacePackage,
};
