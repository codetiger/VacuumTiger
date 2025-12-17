//! Navigation features for high-level robot operations.
//!
//! Features interact with the navigation system **exclusively through the target stack API**
//! (`push_target`, `clear_targets`). They do not directly control motion.
//!
//! # Architecture
//!
//! ```text
//! ┌──────────────────────────────────────────────────────────┐
//! │                     FEATURES                              │
//! │  ┌────────────┐ ┌────────────┐ ┌────────────────────┐   │
//! │  │  Mapping   │ │  Docking   │ │     Sweeping       │   │
//! │  └─────┬──────┘ └─────┬──────┘ └─────────┬──────────┘   │
//! │        │              │                   │              │
//! │        └──────────────┼───────────────────┘              │
//! │                       │ push_target()                    │
//! │                       ▼                                  │
//! │  ┌──────────────────────────────────────────────────┐   │
//! │  │               NavigationState                     │   │
//! │  │  • Target stack                                   │   │
//! │  │  • Current path (A*)                              │   │
//! │  └──────────────────────────────────────────────────┘   │
//! └──────────────────────────────────────────────────────────┘
//! ```
//!
//! # Available Features
//!
//! - [`MappingFeature`]: Autonomous map building via frontier exploration
//! - (Future) `DockingFeature`: Return to dock with backward approach
//! - (Future) `SweepingFeature`: Coverage cleaning patterns

mod clearance;
mod frontier;
mod mapping;

pub use mapping::{MappingConfig, MappingFeature, MappingState};
