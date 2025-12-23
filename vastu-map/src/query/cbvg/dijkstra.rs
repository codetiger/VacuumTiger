//! Dijkstra's shortest path algorithm for CBVG.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// State for Dijkstra's algorithm priority queue.
#[derive(Clone, Copy, Debug)]
pub struct DijkstraState {
    /// Current path cost (distance).
    pub cost: f32,
    /// Current node index.
    pub node: usize,
}

impl PartialEq for DijkstraState {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost && self.node == other.node
    }
}

impl Eq for DijkstraState {}

impl Ord for DijkstraState {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap (BinaryHeap is max-heap by default)
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for DijkstraState {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Result of a Dijkstra search.
#[derive(Clone, Debug)]
pub struct DijkstraResult {
    /// Sequence of node indices from start to goal.
    pub path: Vec<usize>,
    /// Total path distance.
    pub distance: f32,
}

/// Find shortest path using Dijkstra's algorithm.
///
/// # Arguments
/// * `edges` - Adjacency list: edges[i] = [(neighbor_idx, distance), ...]
/// * `start` - Start node index
/// * `goal` - Goal node index
///
/// # Returns
/// DijkstraResult with path and distance, or None if no path exists.
pub fn dijkstra(edges: &[Vec<(usize, f32)>], start: usize, goal: usize) -> Option<DijkstraResult> {
    let n = edges.len();

    if n == 0 || start >= n || goal >= n {
        return None;
    }

    // Handle same start and goal
    if start == goal {
        return Some(DijkstraResult {
            path: vec![start],
            distance: 0.0,
        });
    }

    // Distance to each node
    let mut dist: Vec<f32> = vec![f32::INFINITY; n];
    let mut prev: Vec<Option<usize>> = vec![None; n];
    dist[start] = 0.0;

    // Priority queue
    let mut heap = BinaryHeap::new();
    heap.push(DijkstraState {
        cost: 0.0,
        node: start,
    });

    while let Some(DijkstraState { cost, node }) = heap.pop() {
        // Skip if we've found a better path
        if cost > dist[node] {
            continue;
        }

        // Found goal
        if node == goal {
            break;
        }

        // Explore neighbors
        for &(neighbor, edge_dist) in &edges[node] {
            let new_dist = dist[node] + edge_dist;
            if new_dist < dist[neighbor] {
                dist[neighbor] = new_dist;
                prev[neighbor] = Some(node);
                heap.push(DijkstraState {
                    cost: new_dist,
                    node: neighbor,
                });
            }
        }
    }

    // Check if goal is reachable
    if prev[goal].is_none() && start != goal {
        return None;
    }

    // Reconstruct path
    let mut path = Vec::new();
    let mut current = goal;

    while current != start {
        path.push(current);
        match prev[current] {
            Some(p) => current = p,
            None => return None, // No path
        }
    }
    path.push(start);
    path.reverse();

    Some(DijkstraResult {
        path,
        distance: dist[goal],
    })
}

/// Find shortest path with early termination if goal is found within max_distance.
///
/// Same as `dijkstra` but stops early if the goal is found.
pub fn dijkstra_with_limit(
    edges: &[Vec<(usize, f32)>],
    start: usize,
    goal: usize,
    max_distance: f32,
) -> Option<DijkstraResult> {
    let n = edges.len();

    if n == 0 || start >= n || goal >= n {
        return None;
    }

    if start == goal {
        return Some(DijkstraResult {
            path: vec![start],
            distance: 0.0,
        });
    }

    let mut dist: Vec<f32> = vec![f32::INFINITY; n];
    let mut prev: Vec<Option<usize>> = vec![None; n];
    dist[start] = 0.0;

    let mut heap = BinaryHeap::new();
    heap.push(DijkstraState {
        cost: 0.0,
        node: start,
    });

    while let Some(DijkstraState { cost, node }) = heap.pop() {
        // Early termination if exceeded max distance
        if cost > max_distance {
            break;
        }

        if cost > dist[node] {
            continue;
        }

        if node == goal {
            // Reconstruct path
            let mut path = Vec::new();
            let mut current = goal;
            while current != start {
                path.push(current);
                current = prev[current]?;
            }
            path.push(start);
            path.reverse();

            return Some(DijkstraResult {
                path,
                distance: cost,
            });
        }

        for &(neighbor, edge_dist) in &edges[node] {
            let new_dist = dist[node] + edge_dist;
            if new_dist < dist[neighbor] && new_dist <= max_distance {
                dist[neighbor] = new_dist;
                prev[neighbor] = Some(node);
                heap.push(DijkstraState {
                    cost: new_dist,
                    node: neighbor,
                });
            }
        }
    }

    None
}

/// Compute distances from a source node to all reachable nodes.
///
/// # Arguments
/// * `edges` - Adjacency list
/// * `source` - Source node index
///
/// # Returns
/// Vector of distances (f32::INFINITY for unreachable nodes).
pub fn dijkstra_distances(edges: &[Vec<(usize, f32)>], source: usize) -> Vec<f32> {
    let n = edges.len();

    if n == 0 || source >= n {
        return vec![f32::INFINITY; n];
    }

    let mut dist: Vec<f32> = vec![f32::INFINITY; n];
    dist[source] = 0.0;

    let mut heap = BinaryHeap::new();
    heap.push(DijkstraState {
        cost: 0.0,
        node: source,
    });

    while let Some(DijkstraState { cost, node }) = heap.pop() {
        if cost > dist[node] {
            continue;
        }

        for &(neighbor, edge_dist) in &edges[node] {
            let new_dist = dist[node] + edge_dist;
            if new_dist < dist[neighbor] {
                dist[neighbor] = new_dist;
                heap.push(DijkstraState {
                    cost: new_dist,
                    node: neighbor,
                });
            }
        }
    }

    dist
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_simple_graph() -> Vec<Vec<(usize, f32)>> {
        // Graph:
        // 0 --1.0-- 1 --1.0-- 2
        // |         |
        // 2.0       1.5
        // |         |
        // 3 --1.0-- 4
        vec![
            vec![(1, 1.0), (3, 2.0)],           // Node 0
            vec![(0, 1.0), (2, 1.0), (4, 1.5)], // Node 1
            vec![(1, 1.0)],                     // Node 2
            vec![(0, 2.0), (4, 1.0)],           // Node 3
            vec![(1, 1.5), (3, 1.0)],           // Node 4
        ]
    }

    #[test]
    fn test_dijkstra_simple() {
        let edges = make_simple_graph();

        // Direct path
        let result = dijkstra(&edges, 0, 1).unwrap();
        assert_eq!(result.path, vec![0, 1]);
        assert!((result.distance - 1.0).abs() < 0.01);

        // Path through intermediate node
        let result = dijkstra(&edges, 0, 2).unwrap();
        assert_eq!(result.path, vec![0, 1, 2]);
        assert!((result.distance - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_dijkstra_same_node() {
        let edges = make_simple_graph();
        let result = dijkstra(&edges, 0, 0).unwrap();
        assert_eq!(result.path, vec![0]);
        assert_eq!(result.distance, 0.0);
    }

    #[test]
    fn test_dijkstra_no_path() {
        // Disconnected graph
        let edges: Vec<Vec<(usize, f32)>> = vec![
            vec![(1, 1.0)], // Node 0 -> 1
            vec![(0, 1.0)], // Node 1 -> 0
            vec![],         // Node 2 (isolated)
        ];

        let result = dijkstra(&edges, 0, 2);
        assert!(result.is_none());
    }

    #[test]
    fn test_dijkstra_with_limit() {
        let edges = make_simple_graph();

        // Within limit
        let result = dijkstra_with_limit(&edges, 0, 1, 5.0).unwrap();
        assert_eq!(result.path, vec![0, 1]);

        // Exceeds limit
        let result = dijkstra_with_limit(&edges, 0, 2, 1.5);
        assert!(result.is_none());
    }

    #[test]
    fn test_dijkstra_distances() {
        let edges = make_simple_graph();
        let dist = dijkstra_distances(&edges, 0);

        assert!((dist[0] - 0.0).abs() < 0.01);
        assert!((dist[1] - 1.0).abs() < 0.01);
        assert!((dist[2] - 2.0).abs() < 0.01);
        assert!((dist[3] - 2.0).abs() < 0.01);
        assert!((dist[4] - 2.5).abs() < 0.01);
    }

    #[test]
    fn test_dijkstra_shortest_path() {
        let edges = make_simple_graph();

        // 0 -> 4: Can go 0->1->4 (2.5) or 0->3->4 (3.0)
        let result = dijkstra(&edges, 0, 4).unwrap();
        assert_eq!(result.path, vec![0, 1, 4]);
        assert!((result.distance - 2.5).abs() < 0.01);
    }

    #[test]
    fn test_dijkstra_empty_graph() {
        let edges: Vec<Vec<(usize, f32)>> = vec![];
        let result = dijkstra(&edges, 0, 1);
        assert!(result.is_none());
    }

    #[test]
    fn test_dijkstra_state_ordering() {
        // Test that lower cost has higher priority
        let state1 = DijkstraState { cost: 1.0, node: 0 };
        let state2 = DijkstraState { cost: 2.0, node: 1 };

        // state1 should come before state2 (lower cost = higher priority)
        assert!(state1 > state2);
    }
}
