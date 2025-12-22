//! Test scenarios for integration tests
//!
//! Contains both YAML-based predefined path tests and
//! autonomous exploration tests.
//!
//! # Running Tests
//!
//! ```bash
//! # Run all integration tests
//! cargo test --features integration-tests -- --nocapture
//!
//! # Run exploration test
//! cargo test --features integration-tests exploration -- --nocapture
//! ```

mod exploration_test;
mod yaml_tests;
