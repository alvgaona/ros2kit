use ros2kit::{Env, LaunchArg};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("Usage: launch <package> <launch_file>");
        std::process::exit(1);
    }

    let package = &args[1];
    let launch_file = &args[2];

    let env = Env::from_env()?;
    let python = env.python();

    let path = env
        .launch_file_path(package, launch_file)
        .ok_or_else(|| anyhow::anyhow!("launch file '{launch_file}' not found in '{package}'"))?;

    println!("Launch file: {}", path.display());
    println!("Python: {}\n", python.display());

    let launch_args: Vec<LaunchArg> =
        ros2kit::launch::parse_launch_args(&python, &path).await?;

    if launch_args.is_empty() {
        println!("No declared arguments.");
    } else {
        println!("Declared arguments:");
        for arg in &launch_args {
            if arg.default_value.is_empty() {
                println!("  {} (required)", arg.name);
            } else {
                println!("  {} (default: {})", arg.name, arg.default_value);
            }
        }
    }

    Ok(())
}
