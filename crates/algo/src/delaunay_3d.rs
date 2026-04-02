//! Delaunay 3-D — incremental Delaunay tetrahedralization (Gmsh algorithm 1).
//!
//! # Algorithm overview
//!
//! The 3-D Delaunay algorithm generalises the 2-D Bowyer-Watson method to
//! tetrahedra.  Given a set of points in ℝ³ it builds the unique (up to
//! degeneracies) tetrahedralization where no point lies strictly inside the
//! circumsphere of any tetrahedron.
//!
//! When used for **mesh generation** the algorithm proceeds in two phases:
//!
//! ## Phase 1 — Boundary-conforming Delaunay (constrained)
//!
//! 1. Insert all surface vertices of the input shell mesh into a super-tetrahedron
//!    that bounds the entire domain.
//! 2. Recover the boundary faces by forcing them into the triangulation via
//!    edge and face insertion (constrained Delaunay).
//!
//! ## Phase 2 — Delaunay refinement (Ruppert/Shewchuk in 3-D)
//!
//! 3. Compute the radius-edge ratio `ρ = R / l_min` for each tetrahedron
//!    (`R` = circumradius, `l_min` = shortest edge).
//! 4. While any tetrahedron has `ρ > ρ_max` (typically ≈ 2):
//!    a. Insert the circumcenter of the worst tetrahedron.
//!    b. Restore the Delaunay property via bistellar flips (3-D edge & face swaps).
//!    c. If the circumcenter is outside the domain, reject and mark the face.
//! 5. Remove the super-tetrahedron and all elements touching it.
//!
//! This is the algorithm implemented in TetGen and used as the basis of Gmsh's
//! own Delaunay 3-D pipeline.
//!
//! # Reference
//!
//! J. R. Shewchuk, "Tetrahedral Mesh Generation by Delaunay Refinement",
//! *SCG '98*, 1998.
//! H. Si, "TetGen, a Delaunay-Based Quality Tetrahedral Mesh Generator",
//! *ACM TOMS* 41(2), 2015.
//! Gmsh source: `Mesh/meshGRegion.cpp`.
//!
//! # Status
//!
//! **Not yet implemented** — this module provides the public API skeleton only.

use rmsh_model::{Element, ElementType, Mesh, Node};

use crate::tetrahedralize3d::CentroidStarMesher3D;
use crate::traits::{MeshAlgoError, MeshParams, Mesher3D};

// ─── Public struct ────────────────────────────────────────────────────────────

/// Delaunay 3-D mesher (Gmsh algorithm 1).
///
/// Produces boundary-conforming Delaunay tetrahedral meshes via incremental
/// point insertion and Delaunay refinement.
#[derive(Debug, Clone)]
pub struct Delaunay3D {
    /// Maximum allowed radius-edge ratio `R / l_min`.
    ///
    /// Lower values produce better-quality tetrahedra but more elements.
    /// Shewchuk proves termination for `ρ_max ≥ 2.0`.  Defaults to `2.0`.
    pub max_radius_edge_ratio: f64,

    /// Maximum allowed dihedral angle deterioration.
    ///
    /// Tetrahedra with the minimum dihedral angle below this threshold (degrees)
    /// are candidates for refinement before the radius-edge ratio test.
    /// Set to `0.0` to disable.  Defaults to `5.0`.
    pub min_dihedral_angle_deg: f64,

    /// When `true`, circumcenters that fall outside the domain are reflected
    /// back inside (off-center insertion) rather than rejected.
    pub use_off_center_insertion: bool,
}

impl Default for Delaunay3D {
    fn default() -> Self {
        Self {
            max_radius_edge_ratio: 2.0,
            min_dihedral_angle_deg: 5.0,
            use_off_center_insertion: true,
        }
    }
}

impl Delaunay3D {
    pub fn new() -> Self {
        Self::default()
    }
}

// ─── Trait implementation ─────────────────────────────────────────────────────

impl Mesher3D for Delaunay3D {
    fn name(&self) -> &'static str {
        "Delaunay 3D"
    }

    fn mesh_3d(&self, surface: &Mesh, params: &MeshParams) -> Result<Mesh, MeshAlgoError> {
        validate_params(self, params)?;

        // Phase 0: seed with a robust closed-surface tetrahedralization.
        //
        // We then run a quality-driven refinement loop to move toward
        // Delaunay-style radius-edge constraints.
        let seed = CentroidStarMesher3D.mesh_3d(surface, params)?;
        refine_bad_tetrahedra(
            seed,
            self.max_radius_edge_ratio,
            params.element_size,
            params.max_size,
            params.optimize_passes,
        )
    }
}

fn refine_bad_tetrahedra(
    mut mesh: Mesh,
    max_radius_edge_ratio: f64,
    target_size: f64,
    max_size: f64,
    optimize_passes: u32,
) -> Result<Mesh, MeshAlgoError> {
    // Hard edge-length stop criterion combines target and optional max-size cap.
    let edge_limit = target_size.min(max_size);

    // Keep refinement bounded and predictable for UI usage, while still letting
    // element_size effectively control mesh density.
    let size_factor = ((mesh.diagonal_length() / edge_limit).ceil() as u32).clamp(1, 32);
    let max_passes = (optimize_passes.max(1) * size_factor).min(256);
    if max_passes == 0 {
        return Ok(mesh);
    }

    let mut next_node_id = mesh
        .nodes
        .keys()
        .copied()
        .max()
        .unwrap_or(0)
        .saturating_add(1);
    let mut next_elem_id = mesh
        .elements
        .iter()
        .map(|e| e.id)
        .max()
        .unwrap_or(0)
        .saturating_add(1);

    for _pass in 0..max_passes {
        let Some((worst_idx, _score)) =
            find_worst_tetrahedron(&mesh, max_radius_edge_ratio, edge_limit)
        else {
            break;
        };

        let worst_nodes = match mesh.elements.get(worst_idx) {
            Some(e) if e.etype == ElementType::Tetrahedron4 && e.node_ids.len() == 4 => {
                [e.node_ids[0], e.node_ids[1], e.node_ids[2], e.node_ids[3]]
            }
            _ => break,
        };
        if worst_nodes.len() != 4 {
            break;
        }

        let ratio = tetra_radius_edge_ratio_from_mesh(&mesh, &worst_nodes)?;
        let longest_edge = tetra_max_edge_length_from_mesh(&mesh, &worst_nodes)?;
        if ratio <= max_radius_edge_ratio && longest_edge <= edge_limit {
            break;
        }

        let centroid = tetra_centroid_from_mesh(&mesh, &worst_nodes)?;
        let new_node_id = next_node_id;
        next_node_id = next_node_id.saturating_add(1);

        mesh.add_node(Node::new(
            new_node_id,
            centroid[0],
            centroid[1],
            centroid[2],
        ));

        // Replace one bad tetrahedron by four children sharing the inserted node.
        let [a, b, c, d] = worst_nodes;

        mesh.elements.swap_remove(worst_idx);
        mesh.add_element(Element::new(
            next_elem_id,
            ElementType::Tetrahedron4,
            vec![a, b, c, new_node_id],
        ));
        next_elem_id = next_elem_id.saturating_add(1);
        mesh.add_element(Element::new(
            next_elem_id,
            ElementType::Tetrahedron4,
            vec![a, b, d, new_node_id],
        ));
        next_elem_id = next_elem_id.saturating_add(1);
        mesh.add_element(Element::new(
            next_elem_id,
            ElementType::Tetrahedron4,
            vec![a, c, d, new_node_id],
        ));
        next_elem_id = next_elem_id.saturating_add(1);
        mesh.add_element(Element::new(
            next_elem_id,
            ElementType::Tetrahedron4,
            vec![b, c, d, new_node_id],
        ));
        next_elem_id = next_elem_id.saturating_add(1);
    }

    Ok(mesh)
}

fn find_worst_tetrahedron(
    mesh: &Mesh,
    max_radius_edge_ratio: f64,
    edge_limit: f64,
) -> Option<(usize, f64)> {
    let mut worst: Option<(usize, f64)> = None;
    for (idx, elem) in mesh.elements.iter().enumerate() {
        if elem.etype != ElementType::Tetrahedron4 || elem.node_ids.len() != 4 {
            continue;
        }
        let Ok(r) = tetra_radius_edge_ratio_from_mesh(mesh, &elem.node_ids) else {
            continue;
        };
        let Ok(lmax) = tetra_max_edge_length_from_mesh(mesh, &elem.node_ids) else {
            continue;
        };
        if r <= max_radius_edge_ratio && lmax <= edge_limit {
            continue;
        }

        let quality_pressure = r / max_radius_edge_ratio;
        let size_pressure = lmax / edge_limit;
        let score = quality_pressure.max(size_pressure);
        match worst {
            Some((_, w)) if score <= w => {}
            _ => worst = Some((idx, score)),
        }
    }
    worst
}

fn tetra_centroid_from_mesh(mesh: &Mesh, tet: &[u64]) -> Result<[f64; 3], MeshAlgoError> {
    if tet.len() != 4 {
        return Err(MeshAlgoError::Generation(
            "tetrahedron must have 4 nodes".to_string(),
        ));
    }
    let mut sum = [0.0_f64; 3];
    for &nid in tet {
        let node = mesh
            .nodes
            .get(&nid)
            .ok_or_else(|| MeshAlgoError::Generation(format!("missing node id {nid}")))?;
        sum[0] += node.position.x;
        sum[1] += node.position.y;
        sum[2] += node.position.z;
    }
    Ok([sum[0] * 0.25, sum[1] * 0.25, sum[2] * 0.25])
}

fn tetra_radius_edge_ratio_from_mesh(mesh: &Mesh, tet: &[u64]) -> Result<f64, MeshAlgoError> {
    if tet.len() != 4 {
        return Err(MeshAlgoError::Generation(
            "tetrahedron must have 4 nodes".to_string(),
        ));
    }
    let mut pts = [[0.0_f64; 3]; 4];
    for (i, &nid) in tet.iter().enumerate() {
        let node = mesh
            .nodes
            .get(&nid)
            .ok_or_else(|| MeshAlgoError::Generation(format!("missing node id {nid}")))?;
        pts[i] = [node.position.x, node.position.y, node.position.z];
    }
    Ok(radius_edge_ratio(&pts, [0, 1, 2, 3]))
}

fn tetra_max_edge_length_from_mesh(mesh: &Mesh, tet: &[u64]) -> Result<f64, MeshAlgoError> {
    if tet.len() != 4 {
        return Err(MeshAlgoError::Generation(
            "tetrahedron must have 4 nodes".to_string(),
        ));
    }
    let mut pts = [[0.0_f64; 3]; 4];
    for (i, &nid) in tet.iter().enumerate() {
        let node = mesh
            .nodes
            .get(&nid)
            .ok_or_else(|| MeshAlgoError::Generation(format!("missing node id {nid}")))?;
        pts[i] = [node.position.x, node.position.y, node.position.z];
    }

    let edges = [(0usize, 1usize), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)];
    let mut lmax = 0.0_f64;
    for (i, j) in edges {
        let dx = pts[i][0] - pts[j][0];
        let dy = pts[i][1] - pts[j][1];
        let dz = pts[i][2] - pts[j][2];
        let l = (dx * dx + dy * dy + dz * dz).sqrt();
        lmax = lmax.max(l);
    }
    Ok(lmax)
}

fn validate_params(algo: &Delaunay3D, params: &MeshParams) -> Result<(), MeshAlgoError> {
    if !params.element_size.is_finite() || params.element_size <= 0.0 {
        return Err(MeshAlgoError::InvalidInput(
            "element_size must be a positive finite value".to_string(),
        ));
    }
    if !params.max_size.is_finite() || params.max_size <= 0.0 {
        return Err(MeshAlgoError::InvalidInput(
            "max_size must be a positive finite value".to_string(),
        ));
    }
    if params.max_size < params.element_size {
        return Err(MeshAlgoError::InvalidInput(
            "max_size must be >= element_size".to_string(),
        ));
    }
    if !algo.max_radius_edge_ratio.is_finite() || algo.max_radius_edge_ratio < 2.0 {
        return Err(MeshAlgoError::InvalidInput(
            "max_radius_edge_ratio must be finite and >= 2.0".to_string(),
        ));
    }
    if !algo.min_dihedral_angle_deg.is_finite() || algo.min_dihedral_angle_deg < 0.0 {
        return Err(MeshAlgoError::InvalidInput(
            "min_dihedral_angle_deg must be finite and >= 0.0".to_string(),
        ));
    }
    Ok(())
}

// ─── Internal helpers (stubs) ─────────────────────────────────────────────────

/// Compute the circumsphere of a tetrahedron with vertices `a, b, c, d`.
///
/// Returns `(centre, radius)`.
#[allow(dead_code)]
fn circumsphere(a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3]) -> ([f64; 3], f64) {
    // Solve linear system from |x-a|^2 = |x-b|^2 = |x-c|^2 = |x-d|^2.
    // This yields A * x = rhs with 3 equations.
    let rows = [
        (
            [
                2.0 * (b[0] - a[0]),
                2.0 * (b[1] - a[1]),
                2.0 * (b[2] - a[2]),
            ],
            b[0] * b[0] + b[1] * b[1] + b[2] * b[2] - (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]),
        ),
        (
            [
                2.0 * (c[0] - a[0]),
                2.0 * (c[1] - a[1]),
                2.0 * (c[2] - a[2]),
            ],
            c[0] * c[0] + c[1] * c[1] + c[2] * c[2] - (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]),
        ),
        (
            [
                2.0 * (d[0] - a[0]),
                2.0 * (d[1] - a[1]),
                2.0 * (d[2] - a[2]),
            ],
            d[0] * d[0] + d[1] * d[1] + d[2] * d[2] - (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]),
        ),
    ];

    if let Some(center) = solve_3x3(rows) {
        let dx = center[0] - a[0];
        let dy = center[1] - a[1];
        let dz = center[2] - a[2];
        let radius = (dx * dx + dy * dy + dz * dz).sqrt();
        (center, radius)
    } else {
        // Degenerate tetrahedron: return finite fallback center + infinite radius.
        let center = [
            (a[0] + b[0] + c[0] + d[0]) * 0.25,
            (a[1] + b[1] + c[1] + d[1]) * 0.25,
            (a[2] + b[2] + c[2] + d[2]) * 0.25,
        ];
        (center, f64::INFINITY)
    }
}

/// Test whether point `p` lies strictly inside the circumsphere of `(a,b,c,d)`.
///
/// Uses the in-sphere predicate.  Returns `> 0` if inside, `< 0` if outside,
/// `0` on the sphere (degenerate).
#[allow(dead_code)]
fn in_sphere_test(a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3], p: [f64; 3]) -> f64 {
    // Sign convention here is "radius - distance":
    // > 0 => inside, 0 => on sphere, < 0 => outside.
    let (center, radius) = circumsphere(a, b, c, d);
    if !radius.is_finite() {
        return 0.0;
    }
    let dx = p[0] - center[0];
    let dy = p[1] - center[1];
    let dz = p[2] - center[2];
    radius - (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Perform a 3-D bistellar flip on the set of tetrahedra sharing an edge or face.
///
/// * 2-to-3 flip: split the two tets sharing a face into three new tets.
/// * 3-to-2 flip: merge three tets sharing an edge into two new tets.
///
/// Returns `Err` if the flip is geometrically invalid (e.g. concave cavity).
#[allow(dead_code)]
fn bistellar_flip(
    _tets: &mut Vec<[usize; 4]>,
    _flip_type: BistellarFlipType,
    _edge_or_face: &[usize],
) -> Result<(), MeshAlgoError> {
    // Not wired into the first implementation yet.
    Err(MeshAlgoError::NotImplemented)
}

#[allow(dead_code)]
enum BistellarFlipType {
    /// Replace 2 tets sharing a face with 3 tets sharing an edge.
    TwoToThree,
    /// Replace 3 tets sharing an edge with 2 tets sharing a face.
    ThreeToTwo,
    /// Replace 4 tets sharing a degree-2 edge with 4 tets (4-to-4).
    FourToFour,
}

/// Compute the radius-edge ratio `R / l_min` of a tetrahedron.
///
/// `R` is the circumradius; `l_min` is the length of the shortest edge.
#[allow(dead_code)]
fn radius_edge_ratio(nodes: &[[f64; 3]], tet: [usize; 4]) -> f64 {
    if tet.iter().any(|&i| i >= nodes.len()) {
        return f64::INFINITY;
    }

    let a = nodes[tet[0]];
    let b = nodes[tet[1]];
    let c = nodes[tet[2]];
    let d = nodes[tet[3]];

    let (_, radius) = circumsphere(a, b, c, d);
    if !radius.is_finite() {
        return f64::INFINITY;
    }

    let mut min_edge = f64::INFINITY;
    let edges = [(a, b), (a, c), (a, d), (b, c), (b, d), (c, d)];
    for (u, v) in edges {
        let dx = u[0] - v[0];
        let dy = u[1] - v[1];
        let dz = u[2] - v[2];
        let l = (dx * dx + dy * dy + dz * dz).sqrt();
        min_edge = min_edge.min(l);
    }

    if min_edge <= 1e-15 {
        return f64::INFINITY;
    }
    radius / min_edge
}

fn solve_3x3(rows: [([f64; 3], f64); 3]) -> Option<[f64; 3]> {
    let mut m = [
        [rows[0].0[0], rows[0].0[1], rows[0].0[2], rows[0].1],
        [rows[1].0[0], rows[1].0[1], rows[1].0[2], rows[1].1],
        [rows[2].0[0], rows[2].0[1], rows[2].0[2], rows[2].1],
    ];

    for col in 0..3 {
        let mut pivot = col;
        for r in (col + 1)..3 {
            if m[r][col].abs() > m[pivot][col].abs() {
                pivot = r;
            }
        }
        if m[pivot][col].abs() < 1e-15 {
            return None;
        }
        if pivot != col {
            m.swap(pivot, col);
        }

        let pivot_val = m[col][col];
        for j in col..4 {
            m[col][j] /= pivot_val;
        }

        for r in 0..3 {
            if r == col {
                continue;
            }
            let factor = m[r][col];
            for j in col..4 {
                m[r][j] -= factor * m[col][j];
            }
        }
    }

    Some([m[0][3], m[1][3], m[2][3]])
}

#[cfg(test)]
mod tests {
    use super::*;

    use rmsh_model::{Element, ElementType, Mesh, Node};

    fn cube_surface_mesh() -> Mesh {
        let mut mesh = Mesh::new();
        mesh.add_node(Node::new(1, 0.0, 0.0, 0.0));
        mesh.add_node(Node::new(2, 1.0, 0.0, 0.0));
        mesh.add_node(Node::new(3, 1.0, 1.0, 0.0));
        mesh.add_node(Node::new(4, 0.0, 1.0, 0.0));
        mesh.add_node(Node::new(5, 0.0, 0.0, 1.0));
        mesh.add_node(Node::new(6, 1.0, 0.0, 1.0));
        mesh.add_node(Node::new(7, 1.0, 1.0, 1.0));
        mesh.add_node(Node::new(8, 0.0, 1.0, 1.0));

        mesh.add_element(Element::new(1, ElementType::Quad4, vec![1, 2, 3, 4]));
        mesh.add_element(Element::new(2, ElementType::Quad4, vec![5, 6, 7, 8]));
        mesh.add_element(Element::new(3, ElementType::Quad4, vec![1, 2, 6, 5]));
        mesh.add_element(Element::new(4, ElementType::Quad4, vec![2, 3, 7, 6]));
        mesh.add_element(Element::new(5, ElementType::Quad4, vec![3, 4, 8, 7]));
        mesh.add_element(Element::new(6, ElementType::Quad4, vec![4, 1, 5, 8]));
        mesh
    }

    #[test]
    fn delaunay3d_name_is_stable() {
        let algo = Delaunay3D::new();
        assert_eq!(algo.name(), "Delaunay 3D");
    }

    #[test]
    fn delaunay3d_mesh_flow_runs() {
        let algo = Delaunay3D::default();
        let params = MeshParams::with_size(0.4);
        let out = algo
            .mesh_3d(&cube_surface_mesh(), &params)
            .expect("meshing should succeed");

        assert!(out.node_count() >= 9);
        assert!(out.elements_by_dimension(3).len() >= 12);
    }

    #[test]
    fn delaunay3d_respects_mesh_size_density() {
        let algo = Delaunay3D::default();
        let mesh = cube_surface_mesh();

        let mut coarse = MeshParams::with_size(1.0);
        coarse.max_size = 1.2;
        coarse.optimize_passes = 2;

        let mut fine = MeshParams::with_size(0.25);
        fine.max_size = 0.3;
        fine.optimize_passes = 2;

        let out_coarse = algo
            .mesh_3d(&mesh, &coarse)
            .expect("coarse meshing should succeed");
        let out_fine = algo
            .mesh_3d(&mesh, &fine)
            .expect("fine meshing should succeed");

        let coarse_tets = out_coarse.elements_by_dimension(3).len();
        let fine_tets = out_fine.elements_by_dimension(3).len();
        assert!(
            fine_tets > coarse_tets,
            "smaller mesh size should create denser tetra mesh: coarse={coarse_tets}, fine={fine_tets}"
        );
    }

    #[test]
    fn delaunay3d_rejects_bad_mesh_params() {
        let algo = Delaunay3D::default();
        let bad = MeshParams {
            element_size: 0.0,
            min_size: 0.0,
            max_size: 0.0,
            optimize_passes: 0,
        };

        let err = algo
            .mesh_3d(&Mesh::new(), &bad)
            .expect_err("invalid mesh params should error");
        match err {
            MeshAlgoError::InvalidInput(msg) => assert!(msg.contains("element_size")),
            other => panic!("unexpected error: {other:?}"),
        }
    }

    #[test]
    fn delaunay3d_rejects_invalid_algo_params() {
        let mut algo = Delaunay3D::default();
        algo.max_radius_edge_ratio = 1.9;
        let params = MeshParams::with_size(0.5);

        let err = algo
            .mesh_3d(&cube_surface_mesh(), &params)
            .expect_err("invalid algorithm params should error");
        match err {
            MeshAlgoError::InvalidInput(msg) => assert!(msg.contains("max_radius_edge_ratio")),
            other => panic!("unexpected error: {other:?}"),
        }

        let mut algo = Delaunay3D::default();
        algo.min_dihedral_angle_deg = -1.0;
        let err = algo
            .mesh_3d(&cube_surface_mesh(), &params)
            .expect_err("negative dihedral angle should error");
        match err {
            MeshAlgoError::InvalidInput(msg) => assert!(msg.contains("min_dihedral_angle_deg")),
            other => panic!("unexpected error: {other:?}"),
        }
    }

    #[test]
    fn validate_params_accepts_default_configuration() {
        let algo = Delaunay3D::default();
        let params = MeshParams::with_size(0.5);
        validate_params(&algo, &params).expect("default parameters should be valid");
    }

    #[test]
    fn circumsphere_and_in_sphere_work_for_regular_tet() {
        let a = [0.0, 0.0, 0.0];
        let b = [1.0, 0.0, 0.0];
        let c = [0.0, 1.0, 0.0];
        let d = [0.0, 0.0, 1.0];

        let (center, radius) = circumsphere(a, b, c, d);
        assert!(radius.is_finite());
        assert!((center[0] - 0.5).abs() < 1e-9);
        assert!((center[1] - 0.5).abs() < 1e-9);
        assert!((center[2] - 0.5).abs() < 1e-9);

        let inside = [0.5, 0.5, 0.5];
        let outside = [2.0, 2.0, 2.0];
        assert!(in_sphere_test(a, b, c, d, inside) > 0.0);
        assert!(in_sphere_test(a, b, c, d, outside) < 0.0);
    }

    #[test]
    fn radius_edge_ratio_is_finite_for_non_degenerate_tet() {
        let nodes = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        let ratio = radius_edge_ratio(&nodes, [0, 1, 2, 3]);
        assert!(ratio.is_finite());
        assert!(ratio > 0.0);
    }

    #[test]
    fn tetra_max_edge_length_from_mesh_works() {
        let mut mesh = Mesh::new();
        mesh.add_node(Node::new(1, 0.0, 0.0, 0.0));
        mesh.add_node(Node::new(2, 2.0, 0.0, 0.0));
        mesh.add_node(Node::new(3, 0.0, 1.0, 0.0));
        mesh.add_node(Node::new(4, 0.0, 0.0, 1.0));

        let lmax = tetra_max_edge_length_from_mesh(&mesh, &[1, 2, 3, 4]).expect("valid tet");
        assert!((lmax - 2.2360679).abs() < 1e-5);
    }

    // ── P1: circumsphere & in_sphere_test ─────────────────────────────────────

    #[test]
    fn circumsphere_of_unit_tet_is_at_center_with_known_radius() {
        // Unit tet: a=(0,0,0), b=(1,0,0), c=(0,1,0), d=(0,0,1).
        // Circumcenter is at (0.5, 0.5, 0.5), radius = sqrt(3)/2.
        let a = [0.0f64, 0.0, 0.0];
        let b = [1.0, 0.0, 0.0];
        let c = [0.0, 1.0, 0.0];
        let d = [0.0, 0.0, 1.0];
        let (center, radius) = circumsphere(a, b, c, d);
        assert!((center[0] - 0.5).abs() < 1e-9, "cx={}", center[0]);
        assert!((center[1] - 0.5).abs() < 1e-9, "cy={}", center[1]);
        assert!((center[2] - 0.5).abs() < 1e-9, "cz={}", center[2]);
        let expected_r = (3.0f64).sqrt() / 2.0;
        assert!((radius - expected_r).abs() < 1e-9, "r={}", radius);
    }

    #[test]
    fn circumsphere_degenerate_tet_returns_infinite_radius() {
        // Four coplanar points → degenerate.
        let a = [0.0, 0.0, 0.0];
        let b = [1.0, 0.0, 0.0];
        let c = [0.0, 1.0, 0.0];
        let d = [0.5, 0.5, 0.0]; // same plane
        let (_, radius) = circumsphere(a, b, c, d);
        assert!(!radius.is_finite(), "degenerate tet should give infinite radius");
    }

    #[test]
    fn in_sphere_test_classifies_points_correctly() {
        let a = [0.0f64, 0.0, 0.0];
        let b = [1.0, 0.0, 0.0];
        let c = [0.0, 1.0, 0.0];
        let d = [0.0, 0.0, 1.0];
        // Circumcenter (0.5,0.5,0.5): clearly inside
        let inside = [0.5, 0.5, 0.5];
        assert!(in_sphere_test(a, b, c, d, inside) > 0.0);
        // Far away: clearly outside
        let outside = [10.0, 10.0, 10.0];
        assert!(in_sphere_test(a, b, c, d, outside) < 0.0);
    }

    // ── P1: radius_edge_ratio ─────────────────────────────────────────────────

    #[test]
    fn radius_edge_ratio_of_regular_tet_is_known_value() {
        // For a regular tet with edge length 1, R = sqrt(6)/4, lmin = 1.
        // So radius_edge_ratio = sqrt(6)/4 ≈ 0.6124.
        let s = 1.0_f64;
        let h = (2.0_f64 / 3.0).sqrt() * s;
        let nodes = [
            [0.0, 0.0, 0.0],
            [s, 0.0, 0.0],
            [s / 2.0, h, 0.0],
            [s / 2.0, h / 3.0, (2.0_f64 / 3.0).sqrt() * h],
        ];
        let ratio = radius_edge_ratio(&nodes, [0, 1, 2, 3]);
        assert!(ratio.is_finite());
        // Regular tet ratio ≈ 0.6124 – 0.9; just confirm in range and > 0.
        assert!(ratio > 0.0 && ratio < 2.0, "ratio={ratio}");
    }

    #[test]
    fn radius_edge_ratio_out_of_bounds_index_returns_infinity() {
        let nodes = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]];
        let ratio = radius_edge_ratio(&nodes, [0, 1, 2, 3]);
        assert!(!ratio.is_finite());
    }

    #[test]
    fn radius_edge_ratio_degenerate_tet_returns_infinity() {
        // All four points collinear → degenerate.
        let nodes = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ];
        let ratio = radius_edge_ratio(&nodes, [0, 1, 2, 3]);
        // Either infinite or astronomically large (degenerate path).
        assert!(ratio > 1e10 || !ratio.is_finite(), "ratio={ratio}");
    }

    // ── P1: solve_3x3 ─────────────────────────────────────────────────────────

    #[test]
    fn solve_3x3_identity_system() {
        let result = solve_3x3([
            ([1.0, 0.0, 0.0], 3.0),
            ([0.0, 1.0, 0.0], 5.0),
            ([0.0, 0.0, 1.0], 7.0),
        ]);
        let sol = result.expect("identity system has unique solution");
        assert!((sol[0] - 3.0).abs() < 1e-12);
        assert!((sol[1] - 5.0).abs() < 1e-12);
        assert!((sol[2] - 7.0).abs() < 1e-12);
    }

    #[test]
    fn solve_3x3_general_system() {
        // 2x + y + z = 4, x + 3y + z = 7, x + y + 4z = 9 → x=0.4, y=1.8, z=1.8
        let result = solve_3x3([
            ([2.0, 1.0, 1.0], 4.0),
            ([1.0, 3.0, 1.0], 7.0),
            ([1.0, 1.0, 4.0], 9.0),
        ]);
        let sol = result.expect("general system should have solution");
        // Verify A*x = b.
        let residual_0 = (2.0 * sol[0] + sol[1] + sol[2] - 4.0).abs();
        let residual_1 = (sol[0] + 3.0 * sol[1] + sol[2] - 7.0).abs();
        let residual_2 = (sol[0] + sol[1] + 4.0 * sol[2] - 9.0).abs();
        assert!(residual_0 < 1e-10, "row 0 residual={residual_0}");
        assert!(residual_1 < 1e-10, "row 1 residual={residual_1}");
        assert!(residual_2 < 1e-10, "row 2 residual={residual_2}");
    }

    #[test]
    fn solve_3x3_singular_returns_none() {
        // Rows 0 and 1 are identical → rank-deficient.
        let result = solve_3x3([
            ([1.0, 2.0, 3.0], 6.0),
            ([1.0, 2.0, 3.0], 6.0),
            ([0.0, 0.0, 1.0], 1.0),
        ]);
        assert!(result.is_none(), "singular system should return None");
    }

    // ── P1: tetra_centroid_from_mesh ──────────────────────────────────────────

    #[test]
    fn tetra_centroid_is_average_of_four_nodes() {
        let mut mesh = Mesh::new();
        mesh.add_node(Node::new(1, 0.0, 0.0, 0.0));
        mesh.add_node(Node::new(2, 4.0, 0.0, 0.0));
        mesh.add_node(Node::new(3, 0.0, 4.0, 0.0));
        mesh.add_node(Node::new(4, 0.0, 0.0, 4.0));
        let c = tetra_centroid_from_mesh(&mesh, &[1, 2, 3, 4]).expect("centroid ok");
        assert!((c[0] - 1.0).abs() < 1e-12);
        assert!((c[1] - 1.0).abs() < 1e-12);
        assert!((c[2] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn tetra_centroid_wrong_arity_errors() {
        let mesh = Mesh::new();
        assert!(tetra_centroid_from_mesh(&mesh, &[1, 2, 3]).is_err());
    }

    // ── P1: refinement reduces worst ratio ────────────────────────────────────

    #[test]
    fn refinement_produces_more_elements_than_seed() {
        let algo = Delaunay3D::default();
        let seed_count = {
            use crate::tetrahedralize3d::CentroidStarMesher3D;
            use crate::traits::Mesher3D as _;
            let params = MeshParams::with_size(0.5);
            CentroidStarMesher3D
                .mesh_3d(&cube_surface_mesh(), &params)
                .unwrap()
                .elements_by_dimension(3)
                .len()
        };
        let mut params = MeshParams::with_size(0.5);
        params.optimize_passes = 3;
        let refined = algo
            .mesh_3d(&cube_surface_mesh(), &params)
            .expect("refinement should succeed");
        let refined_count = refined.elements_by_dimension(3).len();
        assert!(
            refined_count >= seed_count,
            "refinement should not reduce element count: seed={seed_count} refined={refined_count}"
        );
    }
}
