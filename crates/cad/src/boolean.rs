use nalgebra::{Point3, Vector3};
use rmsh_model::{Element, ElementType, Mesh, Node};

use crate::shape::Shape;
use crate::tessellate::tessellate;

/// Boolean operation type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanOp {
    Union,
    Intersection,
    Difference,
}

/// Perform an approximate boolean operation on two shapes.
///
/// Both shapes are tessellated at the given `deflection`, then triangles are classified
/// as inside/outside the other solid using a simple ray-casting test.  The surviving
/// triangles are combined into a single [`Mesh`].
///
/// This is a *mesh-level* approximation — not an exact B-Rep boolean.
pub fn boolean(a: &Shape, b: &Shape, op: BooleanOp, deflection: f64) -> Mesh {
    let mesh_a = tessellate(a, deflection);
    let mesh_b = tessellate(b, deflection);

    let tris_a = extract_triangles(&mesh_a);
    let tris_b = extract_triangles(&mesh_b);

    let center_b_inside_a: Vec<bool> = tris_b
        .iter()
        .map(|t| point_inside_mesh(&t.centroid, &tris_a))
        .collect();
    let center_a_inside_b: Vec<bool> = tris_a
        .iter()
        .map(|t| point_inside_mesh(&t.centroid, &tris_b))
        .collect();

    let mut result = Mesh::new();
    let mut next_node: u64 = 1;
    let mut next_elem: u64 = 1;

    // Select triangles according to the operation.
    match op {
        BooleanOp::Union => {
            // A triangles that are NOT inside B  +  B triangles that are NOT inside A
            emit_filtered(&tris_a, &center_a_inside_b, false, &mesh_a, &mut result, &mut next_node, &mut next_elem);
            emit_filtered(&tris_b, &center_b_inside_a, false, &mesh_b, &mut result, &mut next_node, &mut next_elem);
        }
        BooleanOp::Intersection => {
            // A triangles that ARE inside B  +  B triangles that ARE inside A
            emit_filtered(&tris_a, &center_a_inside_b, true, &mesh_a, &mut result, &mut next_node, &mut next_elem);
            emit_filtered(&tris_b, &center_b_inside_a, true, &mesh_b, &mut result, &mut next_node, &mut next_elem);
        }
        BooleanOp::Difference => {
            // A triangles that are NOT inside B  +  B triangles that ARE inside A (flipped)
            emit_filtered(&tris_a, &center_a_inside_b, false, &mesh_a, &mut result, &mut next_node, &mut next_elem);
            emit_filtered(&tris_b, &center_b_inside_a, true, &mesh_b, &mut result, &mut next_node, &mut next_elem);
        }
    }

    result
}

/// Convenience wrappers.
pub fn boolean_union(a: &Shape, b: &Shape, deflection: f64) -> Mesh {
    boolean(a, b, BooleanOp::Union, deflection)
}

pub fn boolean_intersection(a: &Shape, b: &Shape, deflection: f64) -> Mesh {
    boolean(a, b, BooleanOp::Intersection, deflection)
}

pub fn boolean_difference(a: &Shape, b: &Shape, deflection: f64) -> Mesh {
    boolean(a, b, BooleanOp::Difference, deflection)
}

// ---- internal helpers ----

struct TriInfo {
    node_ids: [u64; 3],
    centroid: Point3<f64>,
    // Pre-computed for ray test
    v0: Point3<f64>,
    v1: Point3<f64>,
    v2: Point3<f64>,
}

fn extract_triangles(mesh: &Mesh) -> Vec<TriInfo> {
    let mut tris = Vec::new();
    for e in &mesh.elements {
        match e.etype {
            ElementType::Triangle3 if e.node_ids.len() >= 3 => {
                let p0 = mesh.nodes.get(&e.node_ids[0]).map(|n| n.position);
                let p1 = mesh.nodes.get(&e.node_ids[1]).map(|n| n.position);
                let p2 = mesh.nodes.get(&e.node_ids[2]).map(|n| n.position);
                if let (Some(v0), Some(v1), Some(v2)) = (p0, p1, p2) {
                    let centroid = Point3::from((v0.coords + v1.coords + v2.coords) / 3.0);
                    tris.push(TriInfo {
                        node_ids: [e.node_ids[0], e.node_ids[1], e.node_ids[2]],
                        centroid,
                        v0,
                        v1,
                        v2,
                    });
                }
            }
            ElementType::Quad4 if e.node_ids.len() >= 4 => {
                // Split quad into two triangles: (0,1,2) and (0,2,3)
                let ps: Vec<Option<Point3<f64>>> = (0..4)
                    .map(|i| mesh.nodes.get(&e.node_ids[i]).map(|n| n.position))
                    .collect();
                if let (Some(a), Some(b), Some(c), Some(d)) = (ps[0], ps[1], ps[2], ps[3]) {
                    let c1 = Point3::from((a.coords + b.coords + c.coords) / 3.0);
                    tris.push(TriInfo {
                        node_ids: [e.node_ids[0], e.node_ids[1], e.node_ids[2]],
                        centroid: c1,
                        v0: a,
                        v1: b,
                        v2: c,
                    });
                    let c2 = Point3::from((a.coords + c.coords + d.coords) / 3.0);
                    tris.push(TriInfo {
                        node_ids: [e.node_ids[0], e.node_ids[2], e.node_ids[3]],
                        centroid: c2,
                        v0: a,
                        v1: c,
                        v2: d,
                    });
                }
            }
            _ => {}
        }
    }
    tris
}

/// Simple ray-casting inside/outside test. Shoots a ray in +X direction and counts
/// intersections with the mesh triangles. Odd count = inside.
fn point_inside_mesh(point: &Point3<f64>, triangles: &[TriInfo]) -> bool {
    let ray_origin = *point;
    let ray_dir = Vector3::x();
    let mut count = 0u32;
    for tri in triangles {
        if ray_triangle_intersect(&ray_origin, &ray_dir, &tri.v0, &tri.v1, &tri.v2) {
            count += 1;
        }
    }
    count % 2 == 1
}

/// Möller–Trumbore ray-triangle intersection test.
fn ray_triangle_intersect(
    origin: &Point3<f64>,
    dir: &Vector3<f64>,
    v0: &Point3<f64>,
    v1: &Point3<f64>,
    v2: &Point3<f64>,
) -> bool {
    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let h = dir.cross(&edge2);
    let a = edge1.dot(&h);
    if a.abs() < 1e-12 {
        return false; // ray parallel to triangle
    }
    let f = 1.0 / a;
    let s = origin - v0;
    let u = f * s.dot(&h);
    if !(0.0..=1.0).contains(&u) {
        return false;
    }
    let q = s.cross(&edge1);
    let v = f * dir.dot(&q);
    if v < 0.0 || u + v > 1.0 {
        return false;
    }
    let t = f * edge2.dot(&q);
    t > 1e-12 // intersection must be in front of origin
}

fn emit_filtered(
    tris: &[TriInfo],
    inside_flags: &[bool],
    keep_inside: bool,
    source_mesh: &Mesh,
    result: &mut Mesh,
    next_node: &mut u64,
    next_elem: &mut u64,
) {
    // Map from source node id → result node id (to avoid duplicate nodes)
    let mut node_map: std::collections::HashMap<u64, u64> = std::collections::HashMap::new();

    for (tri, &is_inside) in tris.iter().zip(inside_flags.iter()) {
        if is_inside != keep_inside {
            continue;
        }
        let mut new_nids = Vec::with_capacity(3);
        for &nid in &tri.node_ids {
            let mapped = *node_map.entry(nid).or_insert_with(|| {
                let id = *next_node;
                *next_node += 1;
                if let Some(node) = source_mesh.nodes.get(&nid) {
                    result.add_node(Node::new(id, node.position.x, node.position.y, node.position.z));
                }
                id
            });
            new_nids.push(mapped);
        }
        result.add_element(Element::new(*next_elem, ElementType::Triangle3, new_nids));
        *next_elem += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitive::make_box;
    use crate::transform::translate;

    #[test]
    fn union_of_two_boxes_has_elements() {
        let a = make_box(1.0, 1.0, 1.0);
        let b_shape = make_box(1.0, 1.0, 1.0);
        let b_shifted = translate(&b_shape, Vector3::new(0.5, 0.0, 0.0));
        let result = boolean_union(&a, &b_shifted, 0.5);
        assert!(result.element_count() > 0, "union should produce elements");
    }

    #[test]
    fn intersection_of_overlapping_boxes() {
        let a = make_box(2.0, 2.0, 2.0);
        let b_shape = make_box(2.0, 2.0, 2.0);
        let b_shifted = translate(&b_shape, Vector3::new(1.0, 1.0, 0.0));
        let result = boolean_intersection(&a, &b_shifted, 0.5);
        // Both boxes overlap so intersection should have some triangles
        assert!(result.element_count() > 0, "intersection of overlapping boxes should produce elements");
    }

    #[test]
    fn difference_produces_mesh() {
        let a = make_box(2.0, 2.0, 2.0);
        let b_shape = make_box(1.0, 1.0, 1.0);
        let b_shifted = translate(&b_shape, Vector3::new(0.5, 0.5, 0.5));
        let result = boolean_difference(&a, &b_shifted, 0.5);
        assert!(result.element_count() > 0, "difference should produce elements");
    }

    #[test]
    fn ray_triangle_basic() {
        let v0 = Point3::new(0.0, -1.0, -1.0);
        let v1 = Point3::new(0.0, 1.0, -1.0);
        let v2 = Point3::new(0.0, 0.0, 1.0);
        // Ray from (-1, 0, 0) in +X should hit this YZ-plane triangle
        assert!(ray_triangle_intersect(
            &Point3::new(-1.0, 0.0, 0.0),
            &Vector3::x(),
            &v0,
            &v1,
            &v2,
        ));
        // Ray from (1, 0, 0) in +X should NOT hit (triangle is behind)
        assert!(!ray_triangle_intersect(
            &Point3::new(1.0, 0.0, 0.0),
            &Vector3::x(),
            &v0,
            &v1,
            &v2,
        ));
    }
}
