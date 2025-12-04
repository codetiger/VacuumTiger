//! Build script for SangamIO
//!
//! This script runs at compile time to generate Rust code from protobuf definitions.
//!
//! # What it does
//!
//! 1. Compiles `proto/sangamio.proto` using prost-build
//! 2. Generates Rust structs and enums in `target/*/build/sangam-io-*/out/`
//! 3. Adds clippy suppression for generated code (avoids enum variant name warnings)
//!
//! # Generated Code Location
//!
//! The generated code is included in `src/streaming/wire.rs` via:
//! ```ignore
//! pub mod proto {
//!     include!(concat!(env!("OUT_DIR"), "/sangamio.rs"));
//! }
//! ```
//!
//! # Rebuilding
//!
//! The script automatically reruns when `proto/sangamio.proto` changes.
//! To force regeneration, run `cargo clean` then `cargo build`.

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Configure prost-build to suppress clippy warnings on generated code.
    // The generated enum variants sometimes trigger clippy::enum_variant_names
    // due to naming patterns like SensorValue::BoolVal, SensorValue::U32Val, etc.
    let mut config = prost_build::Config::new();
    config.type_attribute(".", "#[allow(clippy::enum_variant_names)]");

    // Compile proto files into Rust code.
    // Output goes to $OUT_DIR/sangamio.rs
    config.compile_protos(&["proto/sangamio.proto"], &["proto/"])?;

    // Tell Cargo to rerun this script if proto files change.
    // This ensures generated code stays in sync with schema changes.
    println!("cargo:rerun-if-changed=proto/sangamio.proto");

    Ok(())
}
