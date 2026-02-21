use std::collections::BTreeMap;
use std::fs::File;
use std::io::BufWriter;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::Result;
use mcap::records::MessageHeader;
use mcap::write::WriteOptions;
use mcap::Writer;
use tokio::sync::{mpsc, watch};

use crate::ament::Env;

pub struct TopicRecording {
    pub topic: String,
    pub type_name: String,
    pub rx: mpsc::Receiver<Vec<u8>>,
}

pub struct RecordConfig {
    pub output_path: PathBuf,
    pub topics: Vec<TopicRecording>,
}

#[derive(Debug, Clone, Default)]
pub struct RecordStats {
    pub message_count: u64,
    pub bytes_written: u64,
    pub duration: Duration,
}

pub struct RecordResult {
    pub stats_rx: watch::Receiver<RecordStats>,
    pub skipped_topics: Vec<String>,
}

pub struct Recorder {
    active: Arc<AtomicBool>,
    task: Option<tokio::task::JoinHandle<()>>,
}

impl Default for Recorder {
    fn default() -> Self {
        Self::new()
    }
}

impl Recorder {
    pub fn new() -> Self {
        Self {
            active: Arc::new(AtomicBool::new(false)),
            task: None,
        }
    }

    pub fn is_recording(&self) -> bool {
        self.active.load(Ordering::Relaxed)
    }

    pub fn start(&mut self, config: RecordConfig, env: &Env) -> Result<RecordResult> {
        if self.is_recording() {
            anyhow::bail!("already recording");
        }

        let file = File::create(&config.output_path)?;
        let buf = BufWriter::new(file);
        let mut writer = WriteOptions::new()
            .profile("ros2")
            .create(buf)?;

        let mut channels = Vec::new();
        let mut receivers = Vec::new();
        let mut skipped_topics = Vec::new();

        for topic_rec in config.topics {
            let definition = match env.resolve_msg_definition(&topic_rec.type_name) {
                Some(def) => def,
                None => {
                    skipped_topics.push(topic_rec.topic.clone());
                    continue;
                }
            };

            let schema_id = writer.add_schema(
                &topic_rec.type_name,
                "ros2msg",
                definition.as_bytes(),
            )?;

            let channel_id = writer.add_channel(
                schema_id,
                &topic_rec.topic,
                "cdr",
                &BTreeMap::new(),
            )?;

            channels.push(channel_id);
            receivers.push(topic_rec.rx);
        }

        if receivers.is_empty() {
            anyhow::bail!("no topics could be recorded (all .msg definitions unresolvable)");
        }

        let (stats_tx, stats_rx) = watch::channel(RecordStats::default());
        let active = self.active.clone();
        active.store(true, Ordering::Relaxed);

        let handle = tokio::spawn(async move {
            let _ = record_loop(writer, channels, receivers, stats_tx, active).await;
        });

        self.task = Some(handle);

        Ok(RecordResult {
            stats_rx,
            skipped_topics,
        })
    }

    pub fn stop(&mut self) {
        self.active.store(false, Ordering::Relaxed);
        self.task = None;
    }
}

async fn record_loop(
    mut writer: Writer<BufWriter<File>>,
    channels: Vec<u16>,
    mut receivers: Vec<mpsc::Receiver<Vec<u8>>>,
    stats_tx: watch::Sender<RecordStats>,
    active: Arc<AtomicBool>,
) -> Result<()> {
    let start = Instant::now();
    let mut stats = RecordStats::default();
    let mut sequence: u32 = 0;

    loop {
        if !active.load(Ordering::Relaxed) {
            break;
        }

        let mut all_closed = true;
        let mut got_message = false;

        for (i, rx) in receivers.iter_mut().enumerate() {
            match rx.try_recv() {
                Ok(data) => {
                    all_closed = false;
                    got_message = true;

                    let now = SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap_or_default()
                        .as_nanos() as u64;

                    let header = MessageHeader {
                        channel_id: channels[i],
                        sequence,
                        log_time: now,
                        publish_time: now,
                    };

                    stats.bytes_written += data.len() as u64;
                    stats.message_count += 1;
                    stats.duration = start.elapsed();
                    sequence = sequence.wrapping_add(1);

                    let _ = writer.write_to_known_channel(&header, &data);
                    let _ = stats_tx.send(stats.clone());
                }
                Err(mpsc::error::TryRecvError::Empty) => {
                    all_closed = false;
                }
                Err(mpsc::error::TryRecvError::Disconnected) => {}
            }
        }

        if all_closed {
            break;
        }

        if !got_message {
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    }

    active.store(false, Ordering::Relaxed);
    writer.finish()?;
    Ok(())
}
