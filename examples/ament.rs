use ros2kit::Env;

fn main() -> anyhow::Result<()> {
    let env = Env::from_env()?;

    let package_name = std::env::args().nth(1);

    if let Some(ref pkg) = package_name {
        println!("Executables for '{pkg}':");
        for exe in env.executables(pkg) {
            println!("  {} ({:?})", exe.name, exe.language);
        }

        println!("\nLaunch files for '{pkg}':");
        for lf in env.launch_files(pkg) {
            println!("  {} ({:?})", lf.name, lf.format);
        }

        println!("\nInterfaces for '{pkg}':");
        for iface in env.interfaces(pkg) {
            println!("  {} ({:?})", iface.name, iface.kind);
        }
    } else {
        let packages = env.packages();
        println!("Found {} packages:", packages.len());
        for pkg in &packages {
            println!("  {}", pkg.name);
        }
    }

    Ok(())
}
