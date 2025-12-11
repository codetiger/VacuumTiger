//! Map file serialization for occupancy grids.

use std::fs::File;
use std::io::{BufReader, BufWriter, Read, Write};
use std::path::Path;

use super::OccupancyGrid;
use super::config::OccupancyGridConfig;

// Map file format constants
const MAP_MAGIC: u32 = 0x44534D50; // "DSMP" for Dhruva SLAM Map
const MAP_VERSION: u32 = 1;

#[repr(C)]
#[derive(Default)]
struct MapHeader {
    magic: u32,
    version: u32,
    width: u32,
    height: u32,
    resolution: f32,
    origin_x: f32,
    origin_y: f32,
}

/// Save map to a binary file.
pub fn save<P: AsRef<Path>>(grid: &OccupancyGrid, path: P) -> std::io::Result<()> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);

    // Write header
    let header = MapHeader {
        magic: MAP_MAGIC,
        version: MAP_VERSION,
        width: grid.width() as u32,
        height: grid.height() as u32,
        resolution: grid.resolution(),
        origin_x: grid.origin().0,
        origin_y: grid.origin().1,
    };

    // Write header as bytes
    let header_bytes = unsafe {
        std::slice::from_raw_parts(
            &header as *const MapHeader as *const u8,
            std::mem::size_of::<MapHeader>(),
        )
    };
    writer.write_all(header_bytes)?;

    // Write cell data
    let cells = grid.cells();
    let cell_bytes = unsafe {
        std::slice::from_raw_parts(cells.as_ptr() as *const u8, std::mem::size_of_val(cells))
    };
    writer.write_all(cell_bytes)?;

    writer.flush()?;
    Ok(())
}

/// Load map from a binary file.
pub fn load<P: AsRef<Path>>(
    path: P,
    config: OccupancyGridConfig,
) -> std::io::Result<OccupancyGrid> {
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);

    // Read header
    let mut header = MapHeader::default();
    let header_bytes = unsafe {
        std::slice::from_raw_parts_mut(
            &mut header as *mut MapHeader as *mut u8,
            std::mem::size_of::<MapHeader>(),
        )
    };
    reader.read_exact(header_bytes)?;

    // Validate header
    if header.magic != MAP_MAGIC {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "Invalid map file magic number",
        ));
    }

    if header.version != MAP_VERSION {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!("Unsupported map version: {}", header.version),
        ));
    }

    // Read cell data
    let width = header.width as usize;
    let height = header.height as usize;
    let mut cells = vec![0.0f32; width * height];

    let cell_bytes = unsafe {
        std::slice::from_raw_parts_mut(
            cells.as_mut_ptr() as *mut u8,
            cells.len() * std::mem::size_of::<f32>(),
        )
    };
    reader.read_exact(cell_bytes)?;

    Ok(OccupancyGrid::from_raw(
        config,
        cells,
        width,
        height,
        header.origin_x,
        header.origin_y,
    ))
}
