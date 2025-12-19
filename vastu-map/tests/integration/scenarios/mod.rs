//! YAML-based test scenarios for integration tests
//!
//! Each scenario is defined in a YAML file with:
//! - Map reference (PGM + YAML)
//! - Start pose
//! - Path as wheel distance commands
//!
//! # Running Tests
//!
//! ```bash
//! cargo test --features integration-tests -- --nocapture
//! ```

mod yaml_tests;
