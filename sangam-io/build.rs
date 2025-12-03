//! Build script for compiling protobuf definitions

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Configure prost-build to suppress clippy warnings on generated code
    let mut config = prost_build::Config::new();
    config.type_attribute(".", "#[allow(clippy::enum_variant_names)]");

    // Compile proto files
    config.compile_protos(&["proto/sangamio.proto"], &["proto/"])?;

    // Rerun if proto files change
    println!("cargo:rerun-if-changed=proto/sangamio.proto");

    Ok(())
}
