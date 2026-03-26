//! Simple 2-D mesh generation via Bowyer-Watson Delaunay triangulation.
//!
//! # Example
//! ```rust
//! use rmsh_algo::triangulate2d::{Polygon2D, mesh_polygon};
//!
//! // A unit-square polygon
//! let poly = Polygon2D::new(vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
//! let mesh = mesh_polygon(&poly, 0.25).unwrap();
//! assert!(mesh.element_count() > 0);
//! ```

use std::collections::HashMap;

use rmsh_model::{Element, ElementType, Mesh, Node};
use thiserror::Error;

/// Error type for 2-D mesh generation.
#[derive(Error, Debug)]
pub enum MeshError {
    #[error("Mesh generation failed: {0}")]
    Generation(String),
}

/// A 2-D polygon defined by an ordered list of boundary vertices.
pub struct Polygon2D {
    pub vertices: Vec<[f64; 2]>,
}

impl Polygon2D {
    /// Create a new polygon from a list of 2-D vertices (CCW or CW order).
    pub fn new(vertices: Vec<[f64; 2]>) -> Self {
        Self { vertices }
    }

    /// Point-in-polygon test using the ray casting algorithm.
    pub fn contains(&self, p: [f64; 2]) -> bool {
        let n = self.vertices.len();
        let (px, py) = (p[0], p[1]);
        let mut inside = false;
        let mut j = n - 1;
        for i in 0..n {
            let (xi, yi) = (self.vertices[i][0], self.vertices[i][1]);
            let (xj, yj) = (self.vertices[j][0], self.vertices[j][1]);
            if ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi) {
                inside = !inside;
            }
            j = i;
        }
        inside
    }

    /// Axis-aligned bounding box: returns `(min, max)`.
    pub fn bounding_box(&self) -> ([f64; 2], [f64; 2]) {
        let mut min = [f64::MAX; 2];
        let mut max = [f64::MIN; 2];
        for v in &self.vertices {
            min[0] = min[0].min(v[0]);
            min[1] = min[1].min(v[1]);
            max[0] = max[0].max(v[0]);
            max[1] = max[1].max(v[1]);
        }
        (min, max)
    }
}

// ─── Delaunay triangulation (Bowyer-Watson) ──────────────────────────────────

/// Bowyer-Watson incremental Delaunay triangulation.
///
/// Returns a list of triangles as `[i, j, k]` index triples into `pts`.
/// All points must be distinct (within floating-point tolerance).
pub fn triangulate_points(pts: &[[f64; 2]]) -> Vec<[usize; 3]> {
    let n = pts.len();
    if n < 3 {
        return Vec::new();
    }

    // Compute bounding box for super-triangle sizing
    let (mut min_x, mut min_y) = (f64::MAX, f64::MAX);
    let (mut max_x, mut max_y) = (f64::MIN, f64::MIN);
    for p in pts {
        min_x = min_x.min(p[0]);
        min_y = min_y.min(p[1]);
        max_x = max_x.max(p[0]);
        max_y = max_y.max(p[1]);
    }
    let dx = (max_x - min_x).max(1e-9);
    let dy = (max_y - min_y).max(1e-9);
    let d = dx.max(dy);
    let mx = (min_x + max_x) / 2.0;
    let my = (min_y + max_y) / 2.0;

    // Super-triangle covering all input points
    let st0 = [mx - 20.0 * d, my - d];
    let st1 = [mx, my + 20.0 * d];
    let st2 = [mx + 20.0 * d, my - d];

    let mut all: Vec<[f64; 2]> = pts.to_vec();
    let st_start = all.len();
    all.push(st0);
    all.push(st1);
    all.push(st2);

    let mut triangles: Vec<[usize; 3]> = vec![[st_start, st_start + 1, st_start + 2]];

    for i in 0..n {
        let p = pts[i];

        // Find all triangles whose circumcircle contains p
        let bad: Vec<[usize; 3]> = triangles
            .iter()
            .filter(|&&tri| circumcircle_contains(all[tri[0]], all[tri[1]], all[tri[2]], p))
            .copied()
            .collect();

        // Boundary of the polygonal hole formed by removing bad triangles.
        // An edge is on the boundary iff it belongs to exactly one bad triangle.
        let mut boundary: Vec<[usize; 2]> = Vec::new();
        for &tri in &bad {
            let edges = [[tri[0], tri[1]], [tri[1], tri[2]], [tri[2], tri[0]]];
            for edge in edges {
                let shared = bad
                    .iter()
                    .any(|&other| other != tri && tri_has_edge(other, edge[0], edge[1]));
                if !shared {
                    boundary.push(edge);
                }
            }
        }

        triangles.retain(|t| !bad.contains(t));

        for edge in boundary {
            triangles.push([edge[0], edge[1], i]);
        }
    }

    // Drop any triangle that uses super-triangle vertices
    triangles.retain(|t| t[0] < st_start && t[1] < st_start && t[2] < st_start);
    triangles
}

// ─── Polygon meshing ─────────────────────────────────────────────────────────

/// Generate a 2-D triangular mesh inside `polygon` with approximate target
/// edge length `mesh_size`.
///
/// The algorithm:
/// 1. Discretises the boundary edges (respecting `mesh_size`).
/// 2. Scatters interior seed points on a staggered uniform grid.
/// 3. Runs Bowyer-Watson Delaunay triangulation on all points.
/// 4. Discards triangles whose centroid lies outside the polygon.
///
/// Returns a [`Mesh`] with `Triangle3` elements at z = 0.
pub fn mesh_polygon(polygon: &Polygon2D, mesh_size: f64) -> Result<Mesh, MeshError> {
    if polygon.vertices.len() < 3 {
        return Err(MeshError::Generation(
            "Polygon must have at least 3 vertices".to_string(),
        ));
    }
    if mesh_size <= 0.0 {
        return Err(MeshError::Generation(
            "mesh_size must be positive".to_string(),
        ));
    }

    let (bb_min, bb_max) = polygon.bounding_box();

    // ── Boundary points ───────────────────────────────────────────────────
    let mut points: Vec<[f64; 2]> = Vec::new();
    let nv = polygon.vertices.len();
    for i in 0..nv {
        let a = polygon.vertices[i];
        let b = polygon.vertices[(i + 1) % nv];
        let len = dist2(a, b);
        let nseg = ((len / mesh_size).ceil() as usize).max(1);
        for k in 0..nseg {
            let t = k as f64 / nseg as f64;
            points.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
        }
    }

    // ── Interior seed points on staggered grid ────────────────────────────
    let mut iy = 1usize;
    let mut py = bb_min[1] + mesh_size;
    while py < bb_max[1] {
        // Offset every other row by half a cell for better triangle quality
        let offset_x = if iy % 2 == 0 { 0.0 } else { mesh_size * 0.5 };
        let mut px = bb_min[0] + offset_x + mesh_size;
        while px < bb_max[0] {
            if polygon.contains([px, py]) {
                points.push([px, py]);
            }
            px += mesh_size;
        }
        py += mesh_size * 0.866; // sqrt(3)/2 for equilateral row spacing
        iy += 1;
    }

    // ── Deduplicate points that are too close ─────────────────────────────
    let tol = mesh_size * 0.1;
    points = deduplicate(points, tol * tol);

    if points.len() < 3 {
        return Err(MeshError::Generation(
            "Too few distinct points after sampling".to_string(),
        ));
    }

    // ── Delaunay triangulation ────────────────────────────────────────────
    let tris = triangulate_points(&points);

    // ── Build Mesh, keeping only triangles whose centroid is inside polygon ──
    let mut mesh = Mesh::new();
    let mut node_id: u64 = 1;
    let mut elem_id: u64 = 1;
    let mut pt_to_node: HashMap<usize, u64> = HashMap::new();

    for tri in tris {
        let centroid = [
            (points[tri[0]][0] + points[tri[1]][0] + points[tri[2]][0]) / 3.0,
            (points[tri[0]][1] + points[tri[1]][1] + points[tri[2]][1]) / 3.0,
        ];
        if !polygon.contains(centroid) {
            continue;
        }

        let mut nids: Vec<u64> = Vec::with_capacity(3);
        for &pi in &tri {
            let nid = *pt_to_node.entry(pi).or_insert_with(|| {
                let id = node_id;
                node_id += 1;
                mesh.add_node(Node::new(id, points[pi][0], points[pi][1], 0.0));
                id
            });
            nids.push(nid);
        }
        mesh.add_element(Element::new(elem_id, ElementType::Triangle3, nids));
        elem_id += 1;
    }

    if mesh.element_count() == 0 {
        return Err(MeshError::Generation(
            "No interior triangles were generated".to_string(),
        ));
    }

    Ok(mesh)
}

// ─── Internal helpers ─────────────────────────────────────────────────────────

/// Returns `true` if the circumcircle of triangle `(a, b, c)` contains `p`.
///
/// Uses the standard 3×3 determinant criterion, adjusted for triangle
/// orientation so both CW and CCW triangles are handled correctly.
fn circumcircle_contains(a: [f64; 2], b: [f64; 2], c: [f64; 2], p: [f64; 2]) -> bool {
    // Translate so p is at origin
    let ax = a[0] - p[0];
    let ay = a[1] - p[1];
    let bx = b[0] - p[0];
    let by = b[1] - p[1];
    let cx = c[0] - p[0];
    let cy = c[1] - p[1];

    let a2 = ax * ax + ay * ay;
    let b2 = bx * bx + by * by;
    let c2 = cx * cx + cy * cy;

    let det = ax * (by * c2 - cy * b2) - ay * (bx * c2 - cx * b2) + a2 * (bx * cy - by * cx);

    // `det > 0` only means "inside" for CCW triangles.
    // Multiply by the orientation sign to handle CW triangles too.
    let orient = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    det * orient > 0.0
}

fn tri_has_edge(tri: [usize; 3], a: usize, b: usize) -> bool {
    [[tri[0], tri[1]], [tri[1], tri[2]], [tri[2], tri[0]]]
        .iter()
        .any(|e| (e[0] == a && e[1] == b) || (e[0] == b && e[1] == a))
}

fn dist2(a: [f64; 2], b: [f64; 2]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    (dx * dx + dy * dy).sqrt()
}

fn deduplicate(pts: Vec<[f64; 2]>, min_dist_sq: f64) -> Vec<[f64; 2]> {
    let mut result: Vec<[f64; 2]> = Vec::with_capacity(pts.len());
    'outer: for p in pts {
        for q in &result {
            let dx = p[0] - q[0];
            let dy = p[1] - q[1];
            if dx * dx + dy * dy < min_dist_sq {
                continue 'outer;
            }
        }
        result.push(p);
    }
    result
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unit_square_delaunay() {
        // Four corners → 2 triangles (minimum)
        let pts = [[0.0f64, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
        let tris = triangulate_points(&pts);
        assert!(!tris.is_empty(), "should produce at least one triangle");
        // All index values must be valid
        for t in &tris {
            assert!(t[0] < pts.len() && t[1] < pts.len() && t[2] < pts.len());
        }
    }

    #[test]
    fn mesh_unit_square() {
        let poly = Polygon2D::new(vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
        let mesh = mesh_polygon(&poly, 0.25).expect("meshing should succeed");
        assert!(mesh.node_count() > 0);
        assert!(mesh.element_count() > 0);
        // All elements are Triangle3
        for elem in &mesh.elements {
            assert_eq!(elem.etype, ElementType::Triangle3);
            assert_eq!(elem.node_ids.len(), 3);
        }
        // All node IDs in elements must exist in the mesh
        for elem in &mesh.elements {
            for &nid in &elem.node_ids {
                assert!(mesh.nodes.contains_key(&nid));
            }
        }
    }

    #[test]
    fn mesh_l_shape() {
        // L-shaped domain (non-convex)
        let poly = Polygon2D::new(vec![
            [0.0, 0.0],
            [2.0, 0.0],
            [2.0, 1.0],
            [1.0, 1.0],
            [1.0, 2.0],
            [0.0, 2.0],
        ]);
        let mesh = mesh_polygon(&poly, 0.3).expect("L-shape meshing should succeed");
        assert!(mesh.element_count() > 0);
    }

    #[test]
    fn mesh_polygon_produces_planar_2d_mesh() {
        let poly = Polygon2D::new(vec![[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [0.0, 1.0]]);
        let mesh = mesh_polygon(&poly, 0.4).expect("meshing should succeed");

        assert!(mesh.node_count() > 0);
        assert!(mesh.element_count() > 0);

        for node in mesh.nodes.values() {
            assert!(node.position.z.abs() < 1e-12, "2D meshing should keep z=0");
        }

        for elem in &mesh.elements {
            assert_eq!(elem.dimension(), 2, "all elements should be 2D");
            assert_eq!(elem.etype, ElementType::Triangle3);
        }
    }

    #[test]
    fn mesh_rejects_bad_inputs() {
        let poly = Polygon2D::new(vec![[0.0, 0.0], [1.0, 0.0]]);
        assert!(mesh_polygon(&poly, 0.5).is_err(), "< 3 vertices must fail");

        let poly2 = Polygon2D::new(vec![[0.0, 0.0], [1.0, 0.0], [0.5, 1.0]]);
        assert!(
            mesh_polygon(&poly2, -1.0).is_err(),
            "negative size must fail"
        );
    }

    #[test]
    fn point_in_polygon() {
        let poly = Polygon2D::new(vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
        assert!(poly.contains([0.5, 0.5]));
        assert!(!poly.contains([1.5, 0.5]));
        assert!(!poly.contains([-0.1, 0.5]));
    }
}
