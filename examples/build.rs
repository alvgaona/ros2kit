use std::path::PathBuf;

use ros2kit::{BuildConfig, BuildStatus, Builder, CmakeBuildType, PackageSelection};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let workspace_root = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| std::env::current_dir().expect("failed to get current directory"));

    let config = BuildConfig {
        workspace_root,
        packages: PackageSelection::All,
        cmake_args: vec![],
        build_type: CmakeBuildType::Release,
        parallel_jobs: None,
        symlink_install: false,
    };

    let mut builder = Builder::new();
    let result = builder.build(config).await?;

    let mut output_rx = result.output_rx;
    let mut status_rx = result.status_rx;

    loop {
        tokio::select! {
            Some(line) = output_rx.recv() => {
                println!("{line}");
            }
            Ok(()) = status_rx.changed() => {
                match status_rx.borrow_and_update().clone() {
                    BuildStatus::Building { package, progress } => {
                        eprintln!("[status] building {package} ({progress})");
                    }
                    BuildStatus::Finished { success, duration } => {
                        eprintln!("[status] finished: success={success}, duration={duration:.1?}");
                        break;
                    }
                    BuildStatus::Failed { package, error } => {
                        eprintln!("[status] failed: {package} — {error}");
                    }
                }
            }
            else => break,
        }
    }

    while let Some(line) = output_rx.recv().await {
        println!("{line}");
    }

    Ok(())
}
