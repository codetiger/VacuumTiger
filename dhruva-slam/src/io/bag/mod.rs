//! Bag file recording and playback for offline testing.
//!
//! This module provides tools for recording sensor data from SangamIO
//! and replaying it for algorithm development without hardware.
//!
//! # File Format
//!
//! Bag files use a simple binary format with Postcard serialization:
//!
//! ```text
//! ┌──────────────────────────────────────────────────┐
//! │ Header (64 bytes)                                │
//! │ - Magic: "DBAG" (4 bytes)                        │
//! │ - Version: u16                                   │
//! │ - Flags: u16                                     │
//! │ - Start time: u64 (microseconds)                 │
//! │ - End time: u64 (microseconds)                   │
//! │ - Message count: u64                             │
//! │ - Index offset: u64 (0 if no index)              │
//! │ - Reserved: 24 bytes                             │
//! ├──────────────────────────────────────────────────┤
//! │ Message Stream                                   │
//! │ [len:u32][postcard payload]...                   │
//! └──────────────────────────────────────────────────┘
//! ```
//!
//! # Example: Recording
//!
//! ```ignore
//! use dhruva_slam::bag::SangamRecorder;
//! use std::time::Duration;
//!
//! let recorder = SangamRecorder::connect("192.168.68.101:5555", "recording.bag")?;
//! let info = recorder.record_duration(Duration::from_secs(60))?;
//! println!("Recorded {} messages in {:.1}s", info.message_count, info.duration_secs());
//! ```
//!
//! # Example: Playback
//!
//! ```ignore
//! use dhruva_slam::bag::{BagPlayer, SimulatedClient};
//!
//! // Direct playback
//! let mut player = BagPlayer::open("recording.bag")?;
//! for msg in player {
//!     println!("Message at {} us", msg?.timestamp_us());
//! }
//!
//! // Or use SimulatedClient as drop-in replacement for SangamClient
//! let mut client = SimulatedClient::from_bag("recording.bag")?;
//! while let Ok(msg) = client.recv() {
//!     // Same API as SangamClient
//! }
//! ```

mod types;
mod recorder;
mod player;
mod simulated_client;

pub use types::{
    BagHeader, BagInfo, BagMessage, EncoderTicks, SensorStatusMsg,
    HEADER_SIZE, BAG_MAGIC, BAG_VERSION,
};
pub use recorder::BagRecorder;
pub use player::BagPlayer;
pub use simulated_client::SimulatedClient;

#[cfg(test)]
mod test_fixtures;

#[cfg(test)]
pub use test_fixtures::BagTestFixture;
