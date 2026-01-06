# DhruvaNav - Autonomous Exploration Proposal

## Overview

DhruvaNav is a navigation controller for the VacuumTiger robot vacuum. Phase 1 (predefined path testing with map recording) is complete. This proposal focuses on implementing autonomous exploration using frontier-based exploration.

## Current Status (Phase 1 Complete)

- SangamIO TCP/UDP client with sensor streaming
- Wheel encoder odometry with VastuSLAM pose correction
- Rectangular path executor with proportional control
- Occupancy grid mapping using VastuSLAM
- Safety stops on bumper/cliff triggers
- Map saving to `.vastu` and `.svg` formats

## Architecture

```
┌────────────────────────────────────────────────────────────────────┐
│                      DhruvaNav Application                         │
├────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐  ┌─────────────────┐  ┌────────────────────┐  │
│  │    Explorer     │  │  Path Planner   │  │  Motion Controller │  │
│  │ (Frontier-based)│  │    (A*/RRT)     │  │ (Velocity Control) │  │
│  └────────┬────────┘  └────────┬────────┘  └─────────┬──────────┘  │
│           │                    │                     │             │
│  ┌────────▼────────────────────▼─────────────────────▼──────────┐  │
│  │                    State Machine                              │  │
│  │   INIT → EXPLORING → NAVIGATING → EXPLORING → ... → DONE     │  │
│  └──────────────────────────────┬────────────────────────────────┘  │
│                                 │                                   │
│  ┌──────────────────────────────▼────────────────────────────────┐  │
│  │                 Mapper (VastuSLAM)                            │  │
│  │   • Scan matching for pose correction                         │  │
│  │   • Occupancy grid updates                                    │  │
│  │   • Frontier detection from grid                              │  │
│  └──────────────────────────────┬────────────────────────────────┘  │
│                                 │                                   │
│  ┌──────────────────────────────▼────────────────────────────────┐  │
│  │                   SangamIO Client                             │  │
│  │   • TCP: Commands (velocity, stop, actuators)                 │  │
│  │   • UDP: Sensors (lidar @ 5Hz, encoders @ 110Hz)              │  │
│  └───────────────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────────┘
                              │
                              │ Network (Port 5555)
                              ▼
┌────────────────────────────────────────────────────────────────────┐
│                      SangamIO Daemon                               │
│              (Hardware Abstraction Layer)                          │
└────────────────────────────────────────────────────────────────────┘
```

---

## Frontier-Based Exploration Algorithm

### Concept

Frontier-based exploration autonomously maps unknown environments by:
1. Identifying **frontiers** - boundaries between explored (free) and unexplored (unknown) space
2. Selecting the best frontier based on distance, size, and information gain
3. Planning a collision-free path to the selected frontier
4. Navigating to the frontier while updating the map
5. Repeating until no frontiers remain (full coverage)

### Algorithm Flow

```
┌──────────────────────────────────────────────────────────────────┐
│                    EXPLORATION LOOP                               │
├──────────────────────────────────────────────────────────────────┤
│                                                                   │
│   ┌─────────────┐                                                 │
│   │ Update Map  │◄────────────────────────────────────────────┐  │
│   │ (lidar scan)│                                              │  │
│   └──────┬──────┘                                              │  │
│          ▼                                                      │  │
│   ┌──────────────┐    No frontiers    ┌────────────────────┐   │  │
│   │   Detect     │───────────────────►│   DONE: Full       │   │  │
│   │  Frontiers   │                    │   Coverage!        │   │  │
│   └──────┬───────┘                    └────────────────────┘   │  │
│          │ Frontiers exist                                      │  │
│          ▼                                                      │  │
│   ┌──────────────┐                                              │  │
│   │   Score &    │                                              │  │
│   │   Select     │                                              │  │
│   │   Frontier   │                                              │  │
│   └──────┬───────┘                                              │  │
│          ▼                                                      │  │
│   ┌──────────────┐    No path found   ┌────────────────────┐   │  │
│   │  Plan Path   │───────────────────►│ Mark frontier as   │───┘  │
│   │  to Frontier │                    │ unreachable        │      │
│   └──────┬───────┘                    └────────────────────┘      │
│          │ Path found                                             │
│          ▼                                                        │
│   ┌──────────────┐                                                │
│   │  Navigate    │                                                │
│   │  Along Path  │────────────────────────────────────────────────┘
│   └──────────────┘     (continuous map updates during navigation)
│                                                                   │
└──────────────────────────────────────────────────────────────────┘
```

---

## Component Design

### 1. Frontier Detector

Identifies frontiers from the occupancy grid by finding free cells adjacent to unknown cells.

```rust
/// A frontier is a connected region of free cells adjacent to unknown space
pub struct Frontier {
    /// Cells that make up this frontier
    pub cells: Vec<GridCoord>,
    /// Centroid in world coordinates (navigation target)
    pub centroid: WorldPoint,
    /// Size in number of cells
    pub size: usize,
    /// Average distance from robot (for scoring)
    pub distance: f32,
}

pub struct FrontierDetector {
    /// Minimum cells to consider a valid frontier
    min_frontier_size: usize,  // default: 5
    /// Grid resolution for distance calculation
    resolution: f32,
}

impl FrontierDetector {
    /// Detect all frontiers in the current map
    pub fn detect(&self, grid: &GridStorage, robot_pose: Pose2D) -> Vec<Frontier>;

    /// Check if a cell is a frontier cell (free with unknown neighbor)
    fn is_frontier_cell(&self, grid: &GridStorage, coord: GridCoord) -> bool;

    /// Group adjacent frontier cells into connected regions
    fn cluster_frontiers(&self, frontier_cells: Vec<GridCoord>) -> Vec<Frontier>;
}
```

**Frontier Detection Algorithm:**
```
for each cell in grid:
    if cell is FREE and has UNKNOWN neighbor:
        mark as frontier cell

cluster frontier cells using flood-fill:
    for each unvisited frontier cell:
        BFS to find all connected frontier cells
        create Frontier with cells, compute centroid

filter frontiers by minimum size
compute distance from robot to each frontier centroid
```

### 2. Frontier Scorer

Ranks frontiers to determine exploration priority.

```rust
pub struct FrontierScorer {
    /// Weight for distance (closer = better)
    distance_weight: f32,      // default: 1.0
    /// Weight for size (larger = more information gain)
    size_weight: f32,          // default: 0.5
    /// Weight for heading alignment (facing frontier = better)
    alignment_weight: f32,     // default: 0.3
}

impl FrontierScorer {
    /// Score a frontier (higher = better target)
    pub fn score(&self, frontier: &Frontier, robot_pose: Pose2D) -> f32 {
        let distance_score = 1.0 / (1.0 + frontier.distance);
        let size_score = (frontier.size as f32).ln();
        let alignment_score = self.compute_alignment(frontier, robot_pose);

        self.distance_weight * distance_score
            + self.size_weight * size_score
            + self.alignment_weight * alignment_score
    }

    /// Select the best frontier
    pub fn select_best(&self, frontiers: &[Frontier], robot_pose: Pose2D) -> Option<&Frontier>;
}
```

**Scoring Factors:**
| Factor | Rationale |
|--------|-----------|
| Distance | Closer frontiers reduce travel time |
| Size | Larger frontiers reveal more unknown area |
| Alignment | Frontiers in current heading require less rotation |
| Reachability | Penalize previously failed frontiers |

### 3. Cost Map

Pre-computes traversability and wall-distance costs for efficient path planning.

```rust
/// Cost map for path planning with obstacle inflation and wall distance penalty
pub struct CostMap {
    /// Grid dimensions
    width: usize,
    height: usize,
    /// Resolution in meters per cell
    resolution: f32,
    /// Cost values: 0 = free, 255 = lethal obstacle
    costs: Vec<u8>,
    /// Distance to nearest obstacle (in cells) for each cell
    distance_field: Vec<f32>,
}

/// Cost thresholds
pub const COST_FREE: u8 = 0;           // Safe to traverse
pub const COST_NEAR_OBSTACLE: u8 = 50; // Close to wall, penalized
pub const COST_INSCRIBED: u8 = 254;    // Within robot radius, avoid
pub const COST_LETHAL: u8 = 255;       // Obstacle or unknown, blocked

impl CostMap {
    /// Build cost map from occupancy grid
    pub fn from_grid(
        grid: &GridStorage,
        robot_radius: f32,      // 0.17m
        safety_margin: f32,     // 0.05m extra clearance
        wall_penalty_dist: f32, // 0.30m - penalize within this distance
    ) -> Self;

    /// Update cost map incrementally when grid changes
    pub fn update_region(&mut self, grid: &GridStorage, min: GridCoord, max: GridCoord);

    /// Get traversal cost at a grid coordinate
    pub fn cost(&self, coord: GridCoord) -> u8;

    /// Check if line-of-sight is clear (for Theta*)
    pub fn line_of_sight(&self, from: GridCoord, to: GridCoord) -> bool;

    /// Get distance to nearest obstacle (for wall avoidance)
    pub fn obstacle_distance(&self, coord: GridCoord) -> f32;
}
```

**Cost Map Layers:**

```
┌─────────────────────────────────────────────────────────────────┐
│                     COST MAP COMPUTATION                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  1. LETHAL LAYER (cost = 255)                                   │
│     • All cells with CellType: Wall, Cliff, Bump, Unknown       │
│                                                                 │
│  2. INSCRIBED LAYER (cost = 254)                                │
│     • Cells within robot_radius of any lethal cell              │
│     • Robot center cannot enter these cells                     │
│                                                                 │
│  3. INFLATION LAYER (cost = 50-253)                             │
│     • Cells within safety_margin beyond inscribed radius        │
│     • Exponential decay: cost = 50 * exp(-distance/decay)       │
│                                                                 │
│  4. WALL PENALTY LAYER (cost = 1-49)                            │
│     • Cells within wall_penalty_dist of obstacles               │
│     • Linear penalty: closer to wall = higher cost              │
│     • Encourages paths through center of corridors              │
│                                                                 │
│  5. FREE LAYER (cost = 0)                                       │
│     • All remaining traversable cells                           │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Distance Field Computation (Brushfire Algorithm):**

```
1. Initialize distance_field with 0 for obstacles, ∞ for others
2. Add all obstacle cells to queue
3. BFS expansion:
   for each cell in queue:
     for each neighbor:
       new_dist = distance_field[cell] + 1
       if new_dist < distance_field[neighbor]:
         distance_field[neighbor] = new_dist
         add neighbor to queue
4. Convert cell distances to meters: dist_m = dist_cells * resolution
```

### 4. Path Planner (Theta* Algorithm)

Plans any-angle paths that avoid walls with safe clearance.

**Why Theta\* over A\*:**
- A* produces grid-constrained paths (45°/90° turns only)
- Theta* allows any-angle paths → smoother, shorter
- Better suited for differential drive robots
- Wall distance penalty ensures safe clearance

```rust
pub struct PathPlanner {
    /// Pre-computed cost map
    cost_map: CostMap,
    /// Robot radius (for collision checking)
    robot_radius: f32,
    /// Maximum iterations before giving up
    max_iterations: usize,
    /// Wall penalty weight in path cost
    wall_penalty_weight: f32,
}

impl PathPlanner {
    /// Create planner with cost map
    pub fn new(cost_map: CostMap, robot_radius: f32) -> Self;

    /// Plan path using Theta* algorithm
    pub fn plan(
        &self,
        start: Pose2D,
        goal: WorldPoint,
    ) -> Option<PlannedPath>;

    /// Check if direct line-of-sight exists (no obstacles)
    fn line_of_sight(&self, from: GridCoord, to: GridCoord) -> bool;

    /// Compute path cost including wall penalty
    fn path_cost(&self, from: GridCoord, to: GridCoord) -> f32;

    /// Update cost map when occupancy grid changes
    pub fn update_cost_map(&mut self, grid: &GridStorage);
}

pub struct PlannedPath {
    /// Waypoints in world coordinates
    pub waypoints: Vec<WorldPoint>,
    /// Total path length in meters
    pub length: f32,
    /// Total path cost (includes wall penalties)
    pub cost: f32,
}
```

**Theta\* Algorithm:**

```
Theta* extends A* with line-of-sight checks for any-angle paths:

function theta_star(start, goal):
    open_set = priority_queue()
    open_set.push(start, heuristic(start, goal))

    g_score[start] = 0
    parent[start] = start

    while not open_set.empty():
        current = open_set.pop()

        if current == goal:
            return reconstruct_path(goal)

        for neighbor in current.neighbors_8():
            if cost_map.cost(neighbor) >= COST_INSCRIBED:
                continue  // Skip blocked cells

            // KEY DIFFERENCE FROM A*: Check line-of-sight to grandparent
            if line_of_sight(parent[current], neighbor):
                // Path 2: Direct from grandparent (any-angle)
                new_g = g_score[parent[current]] +
                        path_cost(parent[current], neighbor)
                new_parent = parent[current]
            else:
                // Path 1: Through current (grid-constrained)
                new_g = g_score[current] + path_cost(current, neighbor)
                new_parent = current

            if new_g < g_score[neighbor]:
                g_score[neighbor] = new_g
                parent[neighbor] = new_parent
                f_score = new_g + heuristic(neighbor, goal)
                open_set.push(neighbor, f_score)

    return None  // No path found

function path_cost(from, to):
    distance = euclidean_distance(from, to)
    // Sample points along line to accumulate wall penalty
    wall_penalty = integrate_wall_penalty(from, to)
    return distance + wall_penalty_weight * wall_penalty

function line_of_sight(from, to):
    // Bresenham's line algorithm
    for each cell on line from→to:
        if cost_map.cost(cell) >= COST_INSCRIBED:
            return false
    return true
```

**Wall Avoidance via Path Cost:**

```
Path cost = distance + wall_penalty_weight × wall_penalty

wall_penalty for segment (A → B):
    penalty = 0
    for each cell along line A→B:
        dist_to_wall = cost_map.obstacle_distance(cell)
        if dist_to_wall < wall_penalty_dist:
            penalty += (wall_penalty_dist - dist_to_wall) / wall_penalty_dist
    return penalty

Effect: Paths through corridor centers are cheaper than hugging walls
```

### 5. Path Smoother (Differential Drive)

Post-processes Theta* paths for differential drive kinematics.

```rust
pub struct PathSmoother {
    /// Maximum linear velocity (m/s)
    max_linear_vel: f32,
    /// Maximum angular velocity (rad/s)
    max_angular_vel: f32,
    /// Minimum turning radius (m) = max_linear_vel / max_angular_vel
    min_turn_radius: f32,
    /// Lookahead distance for smoothing
    lookahead: f32,
}

impl PathSmoother {
    /// Smooth path for differential drive constraints
    pub fn smooth(&self, path: &PlannedPath) -> SmoothedPath;

    /// Insert arc segments at sharp turns
    fn insert_turn_arcs(&self, waypoints: &[WorldPoint]) -> Vec<PathSegment>;

    /// Check if turn angle exceeds threshold requiring in-place rotation
    fn needs_point_turn(&self, angle: f32) -> bool;
}

pub enum PathSegment {
    /// Straight line segment
    Line { start: WorldPoint, end: WorldPoint },
    /// Arc segment for smooth turns
    Arc { center: WorldPoint, radius: f32, start_angle: f32, end_angle: f32 },
    /// In-place rotation (for sharp turns > 90°)
    PointTurn { position: WorldPoint, from_angle: f32, to_angle: f32 },
}

pub struct SmoothedPath {
    /// Path segments (lines, arcs, point turns)
    pub segments: Vec<PathSegment>,
    /// Estimated traversal time
    pub estimated_time: f32,
}
```

**Smoothing Algorithm:**

```
1. Analyze turn angles at each waypoint
2. For each turn:
   - If angle < 30°: Keep as-is (Theta* already smooth)
   - If 30° < angle < 90°: Insert arc with min_turn_radius
   - If angle > 90°: Insert point turn (rotate in place)
3. Verify smoothed path still avoids obstacles
4. Return segments with velocity profiles
```

**Differential Drive Constraints:**

```
┌─────────────────────────────────────────────────────────────────┐
│              DIFFERENTIAL DRIVE KINEMATICS                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Minimum turn radius: R_min = v_max / ω_max                     │
│                                                                 │
│  For CRL-200S robot:                                            │
│    v_max = 0.2 m/s                                              │
│    ω_max = 0.5 rad/s                                            │
│    R_min = 0.2 / 0.5 = 0.4 m                                    │
│                                                                 │
│  Sharp turns (> 90°):                                           │
│    - Cannot achieve with continuous motion at R_min             │
│    - Must stop, rotate in place, then continue                  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 6. Path Follower

Follows planned paths with obstacle avoidance.

```rust
pub struct PathFollower {
    /// Planned path waypoints
    path: Vec<WorldPoint>,
    /// Current waypoint index
    current_index: usize,
    /// Lookahead distance for pure pursuit
    lookahead_distance: f32,   // default: 0.3m
    /// Waypoint reached tolerance
    waypoint_tolerance: f32,   // default: 0.1m
}

impl PathFollower {
    /// Set a new path to follow
    pub fn set_path(&mut self, path: Vec<WorldPoint>);

    /// Compute velocity command using pure pursuit
    pub fn compute_velocity(&self, robot_pose: Pose2D) -> (f32, f32);

    /// Check if current path segment is blocked by new obstacle
    pub fn is_path_blocked(&self, grid: &GridStorage) -> bool;

    /// Check if path is complete
    pub fn is_complete(&self) -> bool;
}
```

**Pure Pursuit Controller:**
```
1. Find lookahead point on path at distance L from robot
2. Compute curvature to reach lookahead point
3. Convert curvature to (linear, angular) velocity
4. Apply velocity limits
```

### 7. Exploration State Machine

Orchestrates the exploration process.

```rust
pub enum ExplorationState {
    /// Initial state, waiting to start
    Init,
    /// Scanning environment, detecting frontiers
    Scanning,
    /// Planning path to selected frontier
    Planning,
    /// Navigating to frontier
    Navigating,
    /// Recovering from navigation failure
    Recovering,
    /// Exploration complete, no frontiers remain
    Complete,
    /// Safety stop (bumper/cliff triggered)
    SafetyStop,
}

pub struct Explorer {
    state: ExplorationState,
    frontier_detector: FrontierDetector,
    frontier_scorer: FrontierScorer,
    path_planner: PathPlanner,
    path_follower: PathFollower,

    /// Currently targeted frontier
    current_target: Option<Frontier>,
    /// Frontiers that couldn't be reached
    blacklisted_frontiers: HashSet<GridCoord>,
    /// Statistics
    stats: ExplorationStats,
}

pub struct ExplorationStats {
    pub frontiers_visited: usize,
    pub total_distance: f32,
    pub coverage_percent: f32,
    pub time_elapsed: Duration,
}
```

**State Transitions:**

```
┌────────┐
│  Init  │
└───┬────┘
    │ Start exploration
    ▼
┌─────────┐◄─────────────────────────────────────┐
│Scanning │                                       │
└────┬────┘                                       │
     │                                            │
     ├─── No frontiers ──► [Complete]             │
     │                                            │
     │ Frontiers found                            │
     ▼                                            │
┌──────────┐                                      │
│ Planning │                                      │
└────┬─────┘                                      │
     │                                            │
     ├─── No path ──► Blacklist frontier ─────────┤
     │                                            │
     │ Path found                                 │
     ▼                                            │
┌────────────┐                                    │
│ Navigating │                                    │
└─────┬──────┘                                    │
      │                                           │
      ├─── Reached frontier ──────────────────────┘
      │
      ├─── Path blocked ──► [Recovering] ──► [Scanning]
      │
      └─── Bumper/Cliff ──► [SafetyStop]
```

---

## Configuration

```toml
# dhruva.toml

[connection]
robot_ip = "127.0.0.1"
port = 5555
timeout_ms = 5000

[robot]
wheel_base = 0.233
ticks_per_meter = 4464.0
max_linear_vel = 0.2
max_angular_vel = 0.5
robot_radius = 0.17

[exploration]
# Frontier detection
min_frontier_size = 5          # Minimum cells for valid frontier
frontier_update_interval = 2.0 # Seconds between frontier updates

# Frontier scoring weights
distance_weight = 1.0
size_weight = 0.5
alignment_weight = 0.3

# Termination
max_exploration_time = 600     # Maximum time in seconds (0 = unlimited)
return_to_start = true         # Navigate back to start position when done

[path_planning]
# Cost map parameters
safety_margin = 0.05           # Extra clearance beyond robot radius (meters)
wall_penalty_distance = 0.30   # Penalize paths within this distance of walls (meters)
wall_penalty_weight = 2.0      # Weight for wall proximity penalty in path cost

# Theta* algorithm
max_iterations = 50000         # Maximum search iterations before giving up
max_path_length = 20.0         # Maximum path length (meters)

# Path smoothing for differential drive
enable_smoothing = true        # Post-process paths for smooth turns
sharp_turn_threshold = 1.57    # Angle (rad) above which to insert point turn (~90°)
arc_turn_threshold = 0.52      # Angle (rad) above which to insert arc (~30°)

[navigation]
# Pure pursuit path following
lookahead_distance = 0.3       # Lookahead for path following (meters)
waypoint_tolerance = 0.10      # Distance to consider waypoint reached (meters)
goal_tolerance = 0.15          # Distance to consider goal reached (meters)

# Replanning
replan_on_obstacle = true      # Replan if path becomes blocked
replan_interval = 5.0          # Seconds between replan checks during navigation

[output]
map_path = "output/map.vastu"
svg_path = "output/map.svg"
save_interval = 30             # Auto-save every N seconds (0 = disabled)
```

---

## Project Structure

```
dhruva-nav/
├── Cargo.toml
├── dhruva.toml                 # Configuration
├── build.rs                    # Protobuf compilation
├── proto/
│   └── sangamio.proto          # Symlink to sangam-io/proto
└── src/
    ├── main.rs                 # Entry point, main loop
    ├── config.rs               # Configuration loading
    ├── error.rs                # Error types
    │
    ├── client.rs               # SangamIO TCP/UDP client
    ├── odometry.rs             # Wheel encoder odometry
    ├── mapper.rs               # VastuSLAM integration
    │
    ├── planning/               # Path planning module
    │   ├── mod.rs              # Module exports
    │   ├── cost_map.rs         # Cost map with obstacle inflation
    │   ├── theta_star.rs       # Theta* path planner
    │   └── smoother.rs         # Differential drive path smoother
    │
    ├── exploration/            # Exploration module
    │   ├── mod.rs              # Module exports
    │   ├── frontier.rs         # Frontier detection & clustering
    │   ├── scorer.rs           # Frontier scoring & selection
    │   ├── follower.rs         # Pure pursuit path following
    │   └── explorer.rs         # Exploration state machine
    │
    └── path.rs                 # Predefined path (kept for testing)
```

---

## Implementation Phases

### Phase 2A: Cost Map & Distance Field

**Goal:** Build cost map with obstacle inflation and wall distance penalties

**Tasks:**
1. Implement `CostMap` structure with cost layers
2. Implement Brushfire algorithm for distance field computation
3. Add obstacle inflation (inscribed + safety margin)
4. Add wall penalty layer for path cost calculation
5. Implement `line_of_sight()` using Bresenham's algorithm

**Validation:**
- Distance field correctly computed from obstacles
- Inscribed radius blocks cells within robot footprint
- Wall penalty decreases with distance from obstacles
- Line-of-sight correctly detects obstacles

### Phase 2B: Theta* Path Planner

**Goal:** Plan any-angle paths with wall avoidance

**Tasks:**
1. Implement Theta* algorithm with priority queue
2. Integrate wall penalty into path cost function
3. Add path cost computation along line segments
4. Implement goal region detection (not just single cell)

**Validation:**
- Paths are any-angle (not grid-constrained)
- Paths prefer corridor centers over wall-hugging
- Returns None for unreachable goals
- Performance: <500ms for 10m paths

### Phase 2C: Path Smoother

**Goal:** Post-process paths for differential drive kinematics

**Tasks:**
1. Implement turn angle analysis at waypoints
2. Insert arc segments for medium turns (30-90°)
3. Insert point turns for sharp turns (>90°)
4. Verify smoothed path clearance from obstacles

**Validation:**
- Arcs respect minimum turning radius (0.4m)
- Point turns inserted for sharp corners
- Smoothed paths remain collision-free

### Phase 2D: Path Follower

**Goal:** Follow planned paths with pure pursuit

**Tasks:**
1. Implement `PathFollower` with pure pursuit algorithm
2. Handle different segment types (line, arc, point turn)
3. Add path blocking detection
4. Integrate with existing velocity control

**Validation:**
- Robot follows paths smoothly
- Handles transitions between segment types
- Detects when obstacles block planned path
- Reaches waypoints within tolerance

### Phase 2E: Frontier Detection

**Goal:** Detect and cluster frontiers from occupancy grid

**Tasks:**
1. Implement frontier cell detection (Floor + Unknown neighbor)
2. Implement BFS clustering for connected frontiers
3. Compute frontier centroids and sizes
4. Add frontier visualization to SVG output

**Validation:**
- Frontiers correctly identified at free/unknown boundaries
- Clusters are properly formed (no fragmentation)
- Centroid calculation is accurate

### Phase 2F: Exploration Integration

**Goal:** Full autonomous exploration with return-to-start

**Tasks:**
1. Implement `Explorer` state machine
2. Implement `FrontierScorer` with configurable weights
3. Add frontier blacklisting for unreachable targets
4. Implement return-to-start navigation on completion
5. Add exploration statistics tracking

**Validation:**
- Explores entire environment autonomously
- Handles recovery from blocked paths
- Returns to starting position when done
- Terminates when no frontiers remain

---

## Testing Strategy

### Unit Tests

```rust
#[cfg(test)]
mod tests {
    // Frontier detection
    #[test]
    fn test_frontier_cell_detection() { ... }

    #[test]
    fn test_frontier_clustering() { ... }

    // Path planning
    #[test]
    fn test_astar_simple_path() { ... }

    #[test]
    fn test_astar_around_obstacle() { ... }

    #[test]
    fn test_no_path_to_enclosed_goal() { ... }

    // Path following
    #[test]
    fn test_pure_pursuit_straight_line() { ... }

    #[test]
    fn test_pure_pursuit_curve() { ... }
}
```

### Integration Tests with Mock Device

```bash
# Terminal 1: Start SangamIO mock
cd sangam-io
cargo run --release -- --config mock.toml

# Terminal 2: Run exploration
cd dhruva-nav
cargo run --release
```

**Test Scenarios:**

| Scenario | Map | Expected Behavior |
|----------|-----|-------------------|
| Empty room | `empty_room.yaml` | Quick full coverage |
| Single obstacle | `room_with_box.yaml` | Navigate around obstacle |
| Multiple obstacles | `large_room_obstacles.yaml` | Complex frontier selection |
| Maze | `maze.yaml` | Backtracking, recovery |
| Narrow passages | `narrow_corridors.yaml` | Path planning through tight spaces |

---

## Success Criteria

### Functional Requirements

- [ ] Detects frontiers correctly from occupancy grid
- [ ] Plans collision-free paths to frontiers
- [ ] Follows paths with smooth motion
- [ ] Handles path blocking and replanning
- [ ] Explores until no frontiers remain
- [ ] Achieves >90% coverage in test environments
- [ ] Stops safely on bumper/cliff triggers
- [ ] Saves map periodically and on completion

### Performance Requirements

- [ ] Frontier detection < 100ms for 10m x 10m map
- [ ] Path planning < 500ms for 10m path
- [ ] Control loop maintains 20Hz update rate
- [ ] Memory usage < 50MB for typical room

### Robustness Requirements

- [ ] Recovers from navigation failures
- [ ] Handles dynamic obstacles (replanning)
- [ ] Continues after safety stop and resume
- [ ] Handles connection loss gracefully

---

## SIMD Optimizations

DhruvaNav uses `std::simd` (portable SIMD) following VastuSLAM's patterns for high-performance algorithms.

### VastuSLAM SIMD Patterns (Reference)

VastuSLAM provides these SIMD primitives that DhruvaNav can reuse:

```rust
// Core SIMD types (LANES = 4 for ARM NEON / SSE)
use std::simd::{f32x4, i32x4, u8x16};

// VastuSLAM SIMD utilities
use vastu_slam::core::simd::{
    PointCloud,              // SoA point storage
    RotationMatrix4,         // Pre-computed rotation
    transform_points_simd4,  // Batch point transformation
    world_to_grid_simd4,     // Batch coordinate conversion
    distance_squared_simd4,  // Batch distance calculation
    is_valid_coords_simd4,   // Batch bounds checking
};
```

**Key Patterns:**
1. **Structure-of-Arrays (SoA)**: Store X and Y separately for SIMD loads
2. **Padding to LANES**: Always pad arrays to multiples of 4 (f32x4) or 16 (u8x16)
3. **Pre-computed constants**: Broadcast scalars to SIMD vectors outside loops
4. **Scratch buffers**: Reuse allocations for repeated operations

### DhruvaNav SIMD Opportunities

| Component | SIMD Potential | Approach |
|-----------|---------------|----------|
| Cost Map - Distance Field | HIGH | Batch distance comparisons during Brushfire |
| Cost Map - Obstacle Inflation | HIGH | SIMD threshold checks for inflation radius |
| Frontier Detection | HIGH | u8x16 comparison for Floor+Unknown neighbors |
| Path Cost Integration | MEDIUM | Batch wall distance lookups along path |
| Theta* Heuristic | LOW | Single point, limited benefit |
| Path Follower | LOW | Single robot pose |

### 1. SIMD Distance Field (Brushfire)

```rust
use std::simd::{f32x4, cmp::SimdPartialOrd};

const LANES: usize = 4;

/// SIMD-optimized distance field propagation
pub fn propagate_distances_simd(
    distances: &mut [f32],
    width: usize,
    height: usize,
) {
    let sqrt2 = f32x4::splat(1.414);
    let one = f32x4::splat(1.0);

    // Process 4 cells at a time (row-wise)
    for y in 1..height-1 {
        let row_start = y * width;

        for x in (1..width-1).step_by(LANES) {
            if x + LANES > width - 1 { break; }

            // Load current distances
            let curr = f32x4::from_slice(&distances[row_start + x..]);

            // Load neighbor distances (requires careful indexing)
            let north = f32x4::from_slice(&distances[row_start - width + x..]);
            let south = f32x4::from_slice(&distances[row_start + width + x..]);

            // Compute minimum of (neighbor + cost)
            let from_north = north + one;
            let from_south = south + one;

            let min_dist = curr.simd_min(from_north).simd_min(from_south);

            // Store updated distances
            min_dist.copy_to_slice(&mut distances[row_start + x..]);
        }
    }
}
```

### 2. SIMD Frontier Detection

```rust
use std::simd::{u8x16, cmp::SimdPartialEq};

/// Batch check for frontier cells (Floor with Unknown neighbor)
pub fn detect_frontier_cells_simd(
    cell_types: &[u8],
    width: usize,
    frontier_mask: &mut [bool],
) {
    let floor_vec = u8x16::splat(CellType::Floor as u8);
    let unknown_vec = u8x16::splat(CellType::Unknown as u8);

    // Process 16 cells at a time
    for chunk_start in (0..cell_types.len()).step_by(16) {
        if chunk_start + 16 > cell_types.len() { break; }

        let cells = u8x16::from_slice(&cell_types[chunk_start..]);

        // Check if cell is Floor
        let is_floor = cells.simd_eq(floor_vec);

        // For each Floor cell, check neighbors for Unknown
        // (neighbor checking done separately due to irregular access)

        let floor_mask = is_floor.to_bitmask();
        for i in 0..16 {
            if floor_mask & (1 << i) != 0 {
                let idx = chunk_start + i;
                // Check 4 neighbors for Unknown
                frontier_mask[idx] = has_unknown_neighbor(cell_types, idx, width);
            }
        }
    }
}
```

### 3. SIMD Path Cost Integration

```rust
/// SIMD-optimized wall penalty along path segment
pub fn integrate_wall_penalty_simd(
    distance_field: &[f32],
    path_xs: &[f32],  // X coordinates along path (padded to LANES)
    path_ys: &[f32],  // Y coordinates along path
    width: usize,
    resolution: f32,
    penalty_distance: f32,
) -> f32 {
    let n = path_xs.len();
    debug_assert!(n % LANES == 0);

    let inv_res = f32x4::splat(1.0 / resolution);
    let penalty_dist = f32x4::splat(penalty_distance);
    let zero = f32x4::splat(0.0);

    let mut total_penalty = f32x4::splat(0.0);

    for i in (0..n).step_by(LANES) {
        let px = f32x4::from_slice(&path_xs[i..]);
        let py = f32x4::from_slice(&path_ys[i..]);

        // Convert to grid indices
        let gx = (px * inv_res).floor();
        let gy = (py * inv_res).floor();

        // Gather distances (scalar fallback for irregular access)
        let mut dists = [0.0f32; LANES];
        for j in 0..LANES {
            let idx = (gy.to_array()[j] as usize) * width + (gx.to_array()[j] as usize);
            dists[j] = distance_field.get(idx).copied().unwrap_or(0.0);
        }
        let dist_vec = f32x4::from_array(dists);

        // Compute penalty: max(0, penalty_distance - distance) / penalty_distance
        let penalty = (penalty_dist - dist_vec).simd_max(zero) / penalty_dist;
        total_penalty += penalty;
    }

    // Horizontal sum
    total_penalty.reduce_sum()
}
```

### 4. SIMD Centroid Calculation

```rust
/// SIMD-optimized frontier centroid calculation
pub fn compute_centroid_simd(
    xs: &[i32],  // Grid X coordinates (padded)
    ys: &[i32],  // Grid Y coordinates (padded)
    count: usize,
) -> (f32, f32) {
    let n = xs.len();
    debug_assert!(n % LANES == 0);

    let mut sum_x = i32x4::splat(0);
    let mut sum_y = i32x4::splat(0);

    for i in (0..n).step_by(LANES) {
        sum_x += i32x4::from_slice(&xs[i..]);
        sum_y += i32x4::from_slice(&ys[i..]);
    }

    let total_x = sum_x.reduce_sum() as f32;
    let total_y = sum_y.reduce_sum() as f32;

    (total_x / count as f32, total_y / count as f32)
}
```

### 5. Scratch Buffers Pattern

Following VastuSLAM's pattern, reuse allocations:

```rust
/// Pre-allocated buffers for SIMD operations
pub struct PlannerScratch {
    /// World coordinates (padded to LANES)
    pub world_xs: Vec<f32>,
    pub world_ys: Vec<f32>,
    /// Grid coordinates
    pub grid_xs: Vec<i32>,
    pub grid_ys: Vec<i32>,
    /// Distance values
    pub distances: Vec<f32>,
    /// Valid flags
    pub valid: Vec<bool>,
}

impl PlannerScratch {
    pub fn new(capacity: usize) -> Self {
        let padded = capacity.div_ceil(LANES) * LANES;
        Self {
            world_xs: vec![0.0; padded],
            world_ys: vec![0.0; padded],
            grid_xs: vec![0; padded],
            grid_ys: vec![0; padded],
            distances: vec![0.0; padded],
            valid: vec![false; padded],
        }
    }

    pub fn ensure_capacity(&mut self, capacity: usize) {
        let padded = capacity.div_ceil(LANES) * LANES;
        if self.world_xs.len() < padded {
            self.world_xs.resize(padded, 0.0);
            self.world_ys.resize(padded, 0.0);
            self.grid_xs.resize(padded, 0);
            self.grid_ys.resize(padded, 0);
            self.distances.resize(padded, 0.0);
            self.valid.resize(padded, false);
        }
    }
}
```

### Performance Targets with SIMD

| Operation | Scalar Target | SIMD Target | Speedup |
|-----------|---------------|-------------|---------|
| Distance field (100x100) | 50ms | 15ms | 3x |
| Frontier detection | 20ms | 5ms | 4x |
| Path cost (100 samples) | 2ms | 0.5ms | 4x |
| Centroid (500 cells) | 0.5ms | 0.1ms | 5x |

### Implementation Notes

1. **Nightly Rust required**: Uses `#![feature(portable_simd)]`
2. **Fallback**: `std::simd` auto-generates scalar code when SIMD unavailable
3. **ARM NEON**: Primary target (Allwinner A33 in CRL-200S robot)
4. **Testing**: Compare SIMD vs scalar results in unit tests (like VastuSLAM)

---

## Dependencies

```toml
[dependencies]
vastu-slam = { path = "../vastu-slam" }
prost = "0.13"
toml = "0.9"
serde = { version = "1", features = ["derive"] }
tracing = "0.1"
tracing-subscriber = { version = "0.3", features = ["env-filter"] }
thiserror = "2"

# NEW for exploration
priority-queue = "2"           # A* open set
rustc-hash = "2"               # Fast HashSet for visited nodes

[build-dependencies]
prost-build = "0.13"
```

---

## Open Questions

1. **Frontier Refresh Rate:** How often should frontiers be recalculated during navigation?
   - Option A: Every lidar scan (most accurate, highest CPU)
   - Option B: Fixed interval (1-2 seconds)
   - Option C: Only when reaching waypoints

2. **Unknown Space Treatment:** Should the robot plan paths through unknown space?
   - Option A: Only through known-free (safest)
   - Option B: Allow unknown if no free path (more aggressive)

3. **Coverage Metric:** How to measure exploration completeness?
   - Option A: Percentage of grid cells explored
   - Option B: Ratio of explored area to bounding box
   - Option C: No more frontiers above minimum size

4. **Return to Start:** Should robot return to starting position after exploration?
   - Could be useful for docking in vacuum cleaner context

---

## References

- Yamauchi, B. (1997). "A frontier-based approach for autonomous exploration"
- VastuSLAM documentation: `vastu-slam/README.md`
- SangamIO protocol: `sangam-io/proto/sangamio.proto`
