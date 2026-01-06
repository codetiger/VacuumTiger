//! Build script for compiling protobuf definitions

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Compile proto files
    prost_build::compile_protos(&["proto/sangamio.proto"], &["proto/"])?;

    // Rerun if proto files change
    println!("cargo:rerun-if-changed=proto/sangamio.proto");

    Ok(())
}
