//! Command types for inter-thread communication.
//!
//! Commands are sent from the Command Thread to the SLAM Thread via an mpsc channel.
//! Each command includes a oneshot channel for response acknowledgment.
//!
//! Note: Some command variants are defined for planned exploration features.

use std::sync::mpsc;

use crate::algorithms::mapping::OccupancyGrid;

/// Commands sent from Command Thread to SLAM Thread.
#[derive(Debug)]
pub enum SlamCommand {
    /// Start building a new map.
    StartMapping {
        /// Unique map ID for this mapping session.
        map_id: String,
        /// User-friendly map name.
        name: String,
    },

    /// Stop mapping.
    StopMapping,

    /// Clear current in-progress map (reset to empty).
    ClearMap,

    /// Enable a saved map for localization.
    EnableMap {
        /// ID of map to enable.
        map_id: String,
        /// Loaded map data.
        map: OccupancyGrid,
        /// Map name for display.
        name: String,
    },

    /// Emergency stop - disable lidar, set velocity to zero, disable motors.
    EmergencyStop,
}

/// Result of a command execution.
pub type CommandResult = Result<CommandResponse, String>;

/// Response data from command execution.
#[derive(Debug, Clone)]
pub enum CommandResponse {
    /// Mapping started successfully.
    MappingStarted {
        /// Assigned map ID.
        map_id: String,
    },

    /// Mapping stopped.
    MappingStopped,

    /// Map cleared successfully.
    MapCleared,

    /// Map enabled for localization.
    MapEnabled,

    /// Emergency stop completed.
    EmergencyStopped,
}

/// Command with response channel for acknowledgment.
pub struct CommandWithResponse {
    /// The command to execute.
    pub command: SlamCommand,
    /// Channel to send response back.
    pub response_tx: std::sync::mpsc::Sender<CommandResult>,
}

impl std::fmt::Debug for CommandWithResponse {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CommandWithResponse")
            .field("command", &self.command)
            .field("response_tx", &"...")
            .finish()
    }
}

/// Sender end of command channel (held by Command Thread).
pub type CommandSender = mpsc::Sender<CommandWithResponse>;

/// Receiver end of command channel (held by SLAM Thread).
pub type CommandReceiver = mpsc::Receiver<CommandWithResponse>;

/// Create a new command channel pair.
pub fn create_command_channel() -> (CommandSender, CommandReceiver) {
    mpsc::channel()
}

/// Helper to send a command and wait for response.
///
/// This is used by the Command Thread to send commands to the SLAM Thread
/// and wait for acknowledgment.
pub fn send_command_sync(
    sender: &CommandSender,
    command: SlamCommand,
    timeout_ms: u64,
) -> CommandResult {
    use std::time::Duration;

    // Create oneshot-style channel for response
    let (response_tx, response_rx) = mpsc::channel();

    // Send command with response channel
    sender
        .send(CommandWithResponse {
            command,
            response_tx,
        })
        .map_err(|_| "SLAM thread not responding (channel closed)".to_string())?;

    // Wait for response with timeout
    response_rx
        .recv_timeout(Duration::from_millis(timeout_ms))
        .map_err(|e| match e {
            mpsc::RecvTimeoutError::Timeout => "SLAM command timeout".to_string(),
            mpsc::RecvTimeoutError::Disconnected => "SLAM thread disconnected".to_string(),
        })?
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;
    use std::time::Duration;

    #[test]
    fn test_command_channel() {
        let (tx, rx) = create_command_channel();

        // Send a command
        let (response_tx, response_rx) = mpsc::channel();
        tx.send(CommandWithResponse {
            command: SlamCommand::ClearMap,
            response_tx,
        })
        .unwrap();

        // Receive and respond
        let cmd = rx.recv().unwrap();
        assert!(matches!(cmd.command, SlamCommand::ClearMap));
        cmd.response_tx
            .send(Ok(CommandResponse::MapCleared))
            .unwrap();

        // Check response
        let result = response_rx.recv().unwrap();
        assert!(result.is_ok());
    }

    #[test]
    fn test_send_command_sync() {
        let (tx, rx) = create_command_channel();

        // Spawn thread to handle commands
        let handle = thread::spawn(move || {
            while let Ok(cmd) = rx.recv_timeout(Duration::from_millis(100)) {
                match cmd.command {
                    SlamCommand::ClearMap => {
                        cmd.response_tx.send(Ok(CommandResponse::MapCleared)).ok();
                    }
                    _ => {
                        cmd.response_tx.send(Ok(CommandResponse::MapCleared)).ok();
                    }
                }
            }
        });

        // Send command synchronously
        let result = send_command_sync(&tx, SlamCommand::ClearMap, 1000);
        assert!(result.is_ok());

        drop(tx); // Close channel to stop handler thread
        handle.join().unwrap();
    }

    #[test]
    fn test_start_mapping_command() {
        let cmd = SlamCommand::StartMapping {
            map_id: "map_001".to_string(),
            name: "Kitchen".to_string(),
        };

        if let SlamCommand::StartMapping { map_id, name } = cmd {
            assert_eq!(map_id, "map_001");
            assert_eq!(name, "Kitchen");
        } else {
            panic!("Expected StartMapping command");
        }
    }
}
