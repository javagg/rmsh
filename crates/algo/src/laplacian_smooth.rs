//! Laplacian Smoothing — iterative nodal relocation mesh optimizer.
//!
//! # Algorithm overview
//!
//! Laplacian smoothing is the simplest and most widely used mesh quality
//! improvement technique.  Each interior node is moved to the (possibly
//! weighted) centroid of its immediate neighbours:
//!
//! ```text
//! x_i ← (1 - ω) · x_i  +  ω · (1/N · Σ_j x_j)
//! ```
//!
//! where the sum is over the `N` nodes adjacent to node `i`, and `ω ∈ (0, 1]`
//! is the relaxation factor.
//!
//! The standard (unweighted) Laplacian is fast and robust but may shrink the
//! mesh or produce poor results near convex/concave regions.  Improved variants
//! include:
//!
//! * **Taubin smoothing** (λ/μ method): alternating positive and negative
//!   relaxation steps to prevent shrinkage.
//! * **Weighted Laplacian**: weight neighbours by inverse edge length, area,
//!   or cotangent to improve triangle quality.
//! * **Constrained / boundary-locked** Laplacian: skip boundary nodes entirely
//!   (the default here when `params.move_boundary_nodes = false`).
//!
//! # Reference
//!
//! G. Taubin, "A Signal Processing Approach to Fair Surface Design", *SIGGRAPH
//! '95*, 1995 (Taubin smoothing).
//! Gmsh source: `Mesh/meshSmooth.cpp`.
//!
//! # Status
//!
//! The basic (unweighted) Laplacian smoother is trivially implementable.
//! The weighted and Taubin variants are left as stubs.

use rmsh_model::{Mesh, Node};

use crate::traits::{MeshAlgoError, MeshOptimizer, OptimizeParams};

// ─── Public struct ────────────────────────────────────────────────────────────

/// Laplacian smoothing optimizer.
///
/// Relocates each interior node toward the centroid of its neighbours to
/// reduce mesh distortion and improve element quality.
#[derive(Debug, Clone)]
pub struct LaplacianSmooth {
    /// Smoothing variant to apply.
    pub variant: LaplacianVariant,

    /// Relaxation factor `ω ∈ (0, 1]`.
    ///
    /// Larger values converge faster but may be less stable.  Defaults to `0.5`.
    pub omega: f64,
}

impl Default for LaplacianSmooth {
    fn default() -> Self {
        Self {
            variant: LaplacianVariant::Uniform,
            omega: 0.5,
        }
    }
}

impl LaplacianSmooth {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_variant(mut self, variant: LaplacianVariant) -> Self {
        self.variant = variant;
        self
    }
}

// ─── Variant enum ─────────────────────────────────────────────────────────────

/// The smoothing weighting scheme.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LaplacianVariant {
    /// Unweighted centroid average (fastest, may shrink mesh).
    #[default]
    Uniform,

    /// Cotangent-weighted (preserves mesh area better for 2-D triangle meshes).
    Cotangent,

    /// Taubin λ/μ scheme: alternating positive (`λ`) and negative (`μ`) steps
    /// to prevent volume shrinkage.
    Taubin {
        /// Positive step size (`λ > 0`).  Typical: `0.5`.
        lambda: u32,
        /// Negative step size (`μ < 0`, stored as negative integer in milliradians
        /// for `Copy` compatibility — cast back to `f64 / 1000.0`).
        mu_milli: i32,
    },
}

// ─── Trait implementation ─────────────────────────────────────────────────────

impl MeshOptimizer for LaplacianSmooth {
    fn name(&self) -> &'static str {
        "Laplacian Smooth"
    }

    fn optimize(&self, mesh: &mut Mesh, params: &OptimizeParams) -> Result<(), MeshAlgoError> {
        let boundary_node_ids = collect_boundary_nodes(mesh);

        for _iter in 0..params.iterations {
            let max_displacement = laplacian_pass(
                mesh,
                &boundary_node_ids,
                self.omega,
                &self.variant,
                params.move_boundary_nodes,
            )?;

            if max_displacement < params.tolerance {
                break;
            }
        }

        Ok(())
    }
}

// ─── Internal helpers ─────────────────────────────────────────────────────────

/// Perform one Laplacian smoothing pass over all (eligible) nodes.
///
/// Returns the maximum node displacement in this pass (used for convergence
/// checking).
fn laplacian_pass(
    mesh: &mut Mesh,
    boundary_ids: &std::collections::HashSet<u64>,
    omega: f64,
    variant: &LaplacianVariant,
    move_boundary: bool,
) -> Result<f64, MeshAlgoError> {
    // Build adjacency: node_id → list of neighbour node IDs
    let adjacency = build_node_adjacency(mesh);

    let mut max_disp = 0.0_f64;
    let node_ids: Vec<u64> = mesh.nodes.keys().copied().collect();

    for id in node_ids {
        if !move_boundary && boundary_ids.contains(&id) {
            continue;
        }
        let neighbors = match adjacency.get(&id) {
            Some(nbrs) if !nbrs.is_empty() => nbrs,
            _ => continue,
        };

        let new_pos = match variant {
            LaplacianVariant::Uniform => uniform_centroid(id, neighbors, mesh)?,
            LaplacianVariant::Cotangent => {
                // TODO: cotangent-weighted centroid
                return Err(MeshAlgoError::NotImplemented);
            }
            LaplacianVariant::Taubin { .. } => {
                // TODO: alternating λ/μ steps
                return Err(MeshAlgoError::NotImplemented);
            }
        };

        // Apply relaxation
        let node = mesh.nodes.get_mut(&id).unwrap();
        let old = [node.position.x, node.position.y, node.position.z];
        let relaxed = [
            (1.0 - omega) * old[0] + omega * new_pos[0],
            (1.0 - omega) * old[1] + omega * new_pos[1],
            (1.0 - omega) * old[2] + omega * new_pos[2],
        ];
        let disp = {
            let dx = relaxed[0] - old[0];
            let dy = relaxed[1] - old[1];
            let dz = relaxed[2] - old[2];
            (dx * dx + dy * dy + dz * dz).sqrt()
        };
        node.position.x = relaxed[0];
        node.position.y = relaxed[1];
        node.position.z = relaxed[2];
        max_disp = max_disp.max(disp);
    }

    Ok(max_disp)
}

/// Compute the unweighted centroid of a node's neighbours.
fn uniform_centroid(_id: u64, neighbors: &[u64], mesh: &Mesh) -> Result<[f64; 3], MeshAlgoError> {
    let n = neighbors.len() as f64;
    let mut sum = [0.0_f64; 3];
    for &nb_id in neighbors {
        let nb = mesh
            .nodes
            .get(&nb_id)
            .ok_or_else(|| MeshAlgoError::Generation(format!("missing neighbour node {nb_id}")))?;
        sum[0] += nb.position.x;
        sum[1] += nb.position.y;
        sum[2] += nb.position.z;
    }
    Ok([sum[0] / n, sum[1] / n, sum[2] / n])
}

/// Build a node-to-neighbours adjacency map from the element connectivity.
fn build_node_adjacency(mesh: &Mesh) -> std::collections::HashMap<u64, Vec<u64>> {
    use std::collections::{HashMap, HashSet};
    let mut adj: HashMap<u64, HashSet<u64>> = HashMap::new();
    for elem in &mesh.elements {
        let nodes = &elem.node_ids;
        for (i, &a) in nodes.iter().enumerate() {
            for &b in nodes.iter().skip(i + 1) {
                adj.entry(a).or_default().insert(b);
                adj.entry(b).or_default().insert(a);
            }
        }
    }
    adj.into_iter()
        .map(|(k, v)| (k, v.into_iter().collect()))
        .collect()
}

/// Collect the IDs of all nodes that lie on the mesh boundary.
///
/// A boundary node is one that belongs to at least one boundary face (a face
/// not shared by two elements).
fn collect_boundary_nodes(mesh: &Mesh) -> std::collections::HashSet<u64> {
    use std::collections::HashMap;

    // Count how many elements share each face
    let mut face_count: HashMap<[u64; 3], u32> = HashMap::new();
    for elem in &mesh.elements {
        for face in element_faces(&elem.node_ids) {
            *face_count.entry(face).or_insert(0) += 1;
        }
    }

    let mut boundary_nodes = std::collections::HashSet::new();
    for (face, count) in face_count {
        if count == 1 {
            for id in face {
                boundary_nodes.insert(id);
            }
        }
    }
    boundary_nodes
}

/// Return the triangular faces of an element given its node list.
///
/// For Tet4: 4 faces; for Tri3: 1 face (itself).  Other element types
/// are approximated by the first three nodes.
fn element_faces(nodes: &[u64]) -> Vec<[u64; 3]> {
    match nodes.len() {
        4 => {
            // Tet4: four faces
            let [a, b, c, d] = [nodes[0], nodes[1], nodes[2], nodes[3]];
            let mut faces = vec![[a, b, c], [a, b, d], [a, c, d], [b, c, d]];
            for f in &mut faces {
                f.sort_unstable();
            }
            faces
        }
        3 => {
            let mut f = [nodes[0], nodes[1], nodes[2]];
            f.sort_unstable();
            vec![f]
        }
        _ => {
            // Fallback: emit all combinations of 3 nodes from the first 4
            if nodes.len() >= 3 {
                let mut f = [nodes[0], nodes[1], nodes[2]];
                f.sort_unstable();
                vec![f]
            } else {
                vec![]
            }
        }
    }
}

// Bring Node into scope for the position field access — verify it compiles.
const _: fn() = || {
    let _: fn(&Node) -> [f64; 3] = |n| [n.position.x, n.position.y, n.position.z];
};
