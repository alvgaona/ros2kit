// This example is for illustration purposes only and will not run as-is.
// It demonstrates how to use the ros2kit recording API to record topic data
// into an MCAP file. ros2kit handles all the MCAP writing internally — the
// caller only needs to push raw CDR-encoded bytes through a channel.
// The output file can be inspected with Foxglove Studio or the `mcap` CLI.
//
// In a real application, the `tx` side of each channel would be fed by a
// ROS 2 subscription (e.g., via rclrs or a raw DDS binding) that forwards
// incoming CDR-encoded messages.

use std::path::PathBuf;
use std::time::Duration;

use ros2kit::{Env, RecordConfig, Recorder, TopicRecording};
use tokio::sync::mpsc;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let output_path = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("output.mcap"));

    // Requires AMENT_PREFIX_PATH to be set so that message definitions
    // (e.g., std_msgs/String.msg) can be resolved.
    let env = Env::from_env()?;

    // Create a channel for each topic you want to record.
    // The tx side is where you push raw CDR-encoded bytes from your
    // ROS 2 subscription callback; the rx side is consumed by the Recorder.
    let (tx, rx) = mpsc::channel::<Vec<u8>>(64);

    let config = RecordConfig {
        output_path: output_path.clone(),
        topics: vec![TopicRecording {
            topic: "/chatter".to_string(),
            type_name: "std_msgs/String".to_string(),
            rx,
        }],
    };

    let mut recorder = Recorder::new();
    let result = recorder.start(config, &env)?;

    // Topics whose .msg definitions could not be found are skipped.
    if !result.skipped_topics.is_empty() {
        eprintln!("Skipped topics (unresolvable): {:?}", result.skipped_topics);
    }

    let mut stats_rx = result.stats_rx;

    // Simulate a producer sending messages into the channel.
    // In a real application this would be a ROS 2 subscription forwarding
    // CDR-encoded bytes via tx.send(cdr_bytes).await.
    let producer = tokio::spawn(async move {
        for i in 0..10 {
            let msg = format!("hello {i}");
            let data = msg.into_bytes();
            if tx.send(data).await.is_err() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    });

    producer.await?;

    // Signal the recorder to flush and finalize the MCAP file.
    recorder.stop();

    if let Ok(true) = stats_rx.has_changed() {
        let stats = stats_rx.borrow_and_update();
        eprintln!(
            "Recorded {} messages ({} bytes) in {:.1?}",
            stats.message_count, stats.bytes_written, stats.duration
        );
    }

    eprintln!("Written to {}", output_path.display());
    Ok(())
}
