//! Navigation module for goal-based path planning and execution.
//!
//! This module provides the fundamental navigation primitives used by all
//! robot operations (mapping, sweeping, docking, user go-to).
//!
//! # Architecture
//!
//! ```text
//! Features (push targets)          ← WHAT to do
//!     │
//!     │ push_target()
//!     ▼
//! NavigationState                  ← Holds targets + path
//!     │ target_stack
//!     │ current_path
//!     │
//!     ▼
//! PathPlanner (A*)                 ← HOW to reach target
//! ```
//!
//! # Key Types
//!
//! - [`NavTarget`]: A navigation goal with position, optional heading, and movement direction
//! - [`Path`]: A planned path from robot to target (output of A*)
//! - [`NavigationState`]: Target stack and current path with publish-on-change flags

pub mod features;
mod navigator;
mod path;
mod state;
mod target;

pub use navigator::{Navigator, NavigatorConfig};
pub use path::{Path, Waypoint};
pub use state::{NavState, NavigationState, PathFailureReason};
pub use target::{MovementDirection, NavTarget, NavTargetSource, NavTargetType};
