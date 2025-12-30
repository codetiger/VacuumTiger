//! Native .vastu binary format for map persistence.
//!
//! Format:
//! - Header (32 bytes):
//!   - Magic: "VASTU" (5 bytes)
//!   - Version: u8 (1 byte)
//!   - Width: u32 (4 bytes, little-endian)
//!   - Height: u32 (4 bytes, little-endian)
//!   - Resolution: f32 (4 bytes, little-endian)
//!   - Origin X: f32 (4 bytes, little-endian)
//!   - Origin Y: f32 (4 bytes, little-endian)
//!   - Reserved: 6 bytes
//! - Cell data: width * height bytes (1 byte per cell)

use crate::core::{Cell, CellType, WorldPoint};
use crate::grid::GridStorage;
use std::io::{Read, Write};
use std::path::Path;

/// Magic bytes for .vastu format
const MAGIC: &[u8; 5] = b"VASTU";

/// Current format version
const VERSION: u8 = 1;

/// Header size in bytes
const HEADER_SIZE: usize = 32;

/// Error type for I/O operations
#[derive(Debug, Clone)]
pub enum IoError {
    /// File I/O error
    Io(String),
    /// Invalid format
    InvalidFormat(String),
    /// Version mismatch
    VersionMismatch {
        /// Expected format version
        expected: u8,
        /// Found format version
        found: u8,
    },
}

impl std::fmt::Display for IoError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            IoError::Io(msg) => write!(f, "I/O error: {}", msg),
            IoError::InvalidFormat(msg) => write!(f, "Invalid format: {}", msg),
            IoError::VersionMismatch { expected, found } => {
                write!(
                    f,
                    "Version mismatch: expected {}, found {}",
                    expected, found
                )
            }
        }
    }
}

impl std::error::Error for IoError {}

/// Save a grid to .vastu binary format
pub fn save_vastu(storage: &GridStorage, path: &Path) -> Result<(), IoError> {
    let mut file = std::fs::File::create(path).map_err(|e| IoError::Io(e.to_string()))?;

    write_vastu(storage, &mut file)
}

/// Write grid to a writer in .vastu format
pub fn write_vastu<W: Write>(storage: &GridStorage, writer: &mut W) -> Result<(), IoError> {
    // Write header
    let mut header = [0u8; HEADER_SIZE];

    // Magic
    header[0..5].copy_from_slice(MAGIC);

    // Version
    header[5] = VERSION;

    // Width (little-endian u32)
    let width = storage.width() as u32;
    header[6..10].copy_from_slice(&width.to_le_bytes());

    // Height (little-endian u32)
    let height = storage.height() as u32;
    header[10..14].copy_from_slice(&height.to_le_bytes());

    // Resolution (little-endian f32)
    let resolution = storage.resolution();
    header[14..18].copy_from_slice(&resolution.to_le_bytes());

    // Origin X (little-endian f32)
    let origin = storage.origin();
    header[18..22].copy_from_slice(&origin.x.to_le_bytes());

    // Origin Y (little-endian f32)
    header[22..26].copy_from_slice(&origin.y.to_le_bytes());

    // Reserved bytes (zeros)
    // Already zero from initialization

    writer
        .write_all(&header)
        .map_err(|e| IoError::Io(e.to_string()))?;

    // Write cell data
    let mut cell_data = Vec::with_capacity(storage.width() * storage.height());
    for y in 0..storage.height() {
        for x in 0..storage.width() {
            let coord = crate::core::GridCoord::new(x as i32, y as i32);
            let cell = storage.get(coord).unwrap_or_default();
            cell_data.push(encode_cell(&cell));
        }
    }

    writer
        .write_all(&cell_data)
        .map_err(|e| IoError::Io(e.to_string()))?;

    Ok(())
}

/// Load a grid from .vastu binary format
pub fn load_vastu(path: &Path) -> Result<GridStorage, IoError> {
    let mut file = std::fs::File::open(path).map_err(|e| IoError::Io(e.to_string()))?;

    read_vastu(&mut file)
}

/// Read grid from a reader in .vastu format
pub fn read_vastu<R: Read>(reader: &mut R) -> Result<GridStorage, IoError> {
    // Read header
    let mut header = [0u8; HEADER_SIZE];
    reader
        .read_exact(&mut header)
        .map_err(|e| IoError::Io(e.to_string()))?;

    // Verify magic
    if &header[0..5] != MAGIC {
        return Err(IoError::InvalidFormat("Invalid magic bytes".to_string()));
    }

    // Check version
    let version = header[5];
    if version != VERSION {
        return Err(IoError::VersionMismatch {
            expected: VERSION,
            found: version,
        });
    }

    // Read dimensions
    let width = u32::from_le_bytes([header[6], header[7], header[8], header[9]]) as usize;
    let height = u32::from_le_bytes([header[10], header[11], header[12], header[13]]) as usize;

    // Read resolution
    let resolution = f32::from_le_bytes([header[14], header[15], header[16], header[17]]);

    // Read origin
    let origin_x = f32::from_le_bytes([header[18], header[19], header[20], header[21]]);
    let origin_y = f32::from_le_bytes([header[22], header[23], header[24], header[25]]);
    let origin = WorldPoint::new(origin_x, origin_y);

    // Create storage
    let mut storage = GridStorage::new(width, height, resolution, origin);

    // Read cell data
    let mut cell_data = vec![0u8; width * height];
    reader
        .read_exact(&mut cell_data)
        .map_err(|e| IoError::Io(e.to_string()))?;

    // Decode cells
    for (i, &byte) in cell_data.iter().enumerate() {
        let x = (i % width) as i32;
        let y = (i / width) as i32;
        let coord = crate::core::GridCoord::new(x, y);

        let cell = decode_cell(byte);
        if let Some(c) = storage.get_mut(coord) {
            *c.cell_type = cell.cell_type as u8;
            *c.confidence = cell.confidence;
            *c.observation_count = cell.observation_count;
            *c.swept = if cell.swept { 1 } else { 0 };
        }
    }

    Ok(storage)
}

/// Encode a cell to a single byte
fn encode_cell(cell: &Cell) -> u8 {
    let type_bits = cell.cell_type as u8;
    let flags = if cell.swept { 0x80 } else { 0 };
    type_bits | flags
}

/// Decode a cell from a single byte
fn decode_cell(byte: u8) -> Cell {
    let cell_type = match byte & 0x0F {
        0 => CellType::Unknown,
        1 => CellType::Floor,
        2 => CellType::Wall,
        3 => CellType::Cliff,
        4 => CellType::Bump,
        _ => CellType::Unknown,
    };
    let swept = (byte & 0x80) != 0;

    Cell {
        cell_type,
        swept,
        ..Default::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::GridCoord;
    use std::io::Cursor;

    #[test]
    fn test_round_trip() {
        // Create a test grid
        let mut storage = GridStorage::centered(10, 10, 0.1);

        // Set some cells
        storage.set_type(GridCoord::new(5, 5), CellType::Floor);
        storage.set_type(GridCoord::new(6, 5), CellType::Wall);
        storage.set_type(GridCoord::new(7, 5), CellType::Cliff);
        storage.set_type(GridCoord::new(8, 5), CellType::Bump);

        // Mark one cell as swept
        if let Some(cell) = storage.get_mut(GridCoord::new(5, 5)) {
            *cell.swept = 1;
        }

        // Write to buffer
        let mut buffer = Vec::new();
        write_vastu(&storage, &mut buffer).unwrap();

        // Read back
        let mut cursor = Cursor::new(buffer);
        let loaded = read_vastu(&mut cursor).unwrap();

        // Verify dimensions
        assert_eq!(loaded.width(), storage.width());
        assert_eq!(loaded.height(), storage.height());
        assert!((loaded.resolution() - storage.resolution()).abs() < 0.001);

        // Verify cells
        assert_eq!(loaded.get_type(GridCoord::new(5, 5)), CellType::Floor);
        assert_eq!(loaded.get_type(GridCoord::new(6, 5)), CellType::Wall);
        assert_eq!(loaded.get_type(GridCoord::new(7, 5)), CellType::Cliff);
        assert_eq!(loaded.get_type(GridCoord::new(8, 5)), CellType::Bump);
        assert_eq!(loaded.get_type(GridCoord::new(0, 0)), CellType::Unknown);

        // Verify swept flag
        assert!(
            loaded
                .get(GridCoord::new(5, 5))
                .map(|c| c.swept)
                .unwrap_or(false)
        );
    }

    #[test]
    fn test_invalid_magic() {
        // Create full header with wrong magic
        let mut data = Vec::new();
        data.extend_from_slice(b"WRONG"); // Invalid magic (5 bytes)
        data.push(1); // version
        data.extend([0u8; HEADER_SIZE - 6]); // Rest of header

        let mut cursor = Cursor::new(data);
        let result = read_vastu(&mut cursor);

        assert!(matches!(result, Err(IoError::InvalidFormat(_))));
    }

    #[test]
    fn test_version_mismatch() {
        let mut data = Vec::new();
        data.extend_from_slice(MAGIC);
        data.push(99); // Wrong version
        data.extend([0u8; HEADER_SIZE - 6]);

        let mut cursor = Cursor::new(data);
        let result = read_vastu(&mut cursor);

        assert!(matches!(result, Err(IoError::VersionMismatch { .. })));
    }

    #[test]
    fn test_encode_decode_cell() {
        for cell_type in [
            CellType::Unknown,
            CellType::Floor,
            CellType::Wall,
            CellType::Cliff,
            CellType::Bump,
        ] {
            for swept in [false, true] {
                let original = Cell {
                    cell_type,
                    swept,
                    ..Default::default()
                };

                let encoded = encode_cell(&original);
                let decoded = decode_cell(encoded);

                assert_eq!(decoded.cell_type, original.cell_type);
                assert_eq!(decoded.swept, original.swept);
            }
        }
    }
}
