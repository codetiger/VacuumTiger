//! State management for multi-threaded SLAM daemon.
//!
//! This module provides:
//! - `SharedStateHandle`: Thread-safe shared state between SLAM, Command, and Publisher threads
//! - `SlamCommand`: Commands sent from Command Thread to SLAM Thread
//! - Channel types for inter-thread communication

mod commands;
mod shared;

pub use commands::{
    CommandReceiver, CommandResponse, CommandResult, CommandSender, SlamCommand,
    create_command_channel, send_command_sync,
};
pub use shared::{
    CliffState, CurrentMapData, ExplorationMode, MapList, MapSummary, MappingProgressState,
    MappingProgressStateEnum, RobotState, RobotStatus, SensorStatus, SharedStateHandle,
    create_shared_state,
};
