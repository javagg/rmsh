use std::collections::HashMap;

use rmsh_model::{Element, ElementType, Mesh, Node};

use crate::geom::Surface;
use crate::shape::Shape;

/// Tessellate a [`Shape`] into a [`Mesh`].
///
/// `deflection` controls the chord-height tolerance for curved surfaces.
/// Smaller values produce denser meshes. A value of 0.1 is a reasonable default.
pub fn tessellate(shape: &Shape, deflection: f64) -> Mesh {
    let mut mesh = Mesh::new();
    let mut next_node: u64 = 1;
    let mut next_elem: u64 = 1;

    // Map from vertex index to mesh node id (for reusing shared vertices).
    let mut vtx_node: HashMap<usize, u64> = HashMap::new();

    // First, add all CAD vertices as mesh nodes.
    for v in &shape.vertices {
        let nid = next_node;
        next_node += 1;
        mesh.add_node(Node::new(nid, v.point.x, v.point.y, v.point.z));
        vtx_node.insert(v.id, nid);
    }

    // Tessellate each face.
    for face in &shape.faces {
        let wire = &shape.wires[face.outer_wire];
        let face_vertex_ids = collect_face_vertices(shape, wire);

        match &face.surface {
            Surface::Plane { .. } => {
                tessellate_planar_face(
                    &face_vertex_ids,
                    &vtx_node,
                    &mut mesh,
                    &mut next_elem,
                );
            }
            _ => {
                tessellate_curved_face(
                    shape,
                    face,
                    &face_vertex_ids,
                    deflection,
                    &mut mesh,
                    &mut next_node,
                    &mut next_elem,
                );
            }
        }
    }

    mesh
}

/// Collect the ordered vertex indices forming a face boundary from its wire.
fn collect_face_vertices(shape: &Shape, wire: &crate::shape::CadWire) -> Vec<usize> {
    let mut verts: Vec<usize> = Vec::new();
    for (&eid, &forward) in wire.edge_ids.iter().zip(wire.orientations.iter()) {
        let edge = &shape.edges[eid];
        let (a, b) = if forward {
            (edge.start_vertex, edge.end_vertex)
        } else {
            (edge.end_vertex, edge.start_vertex)
        };
        if verts.is_empty() {
            verts.push(a);
        }
        if b != *verts.first().unwrap_or(&usize::MAX) {
            verts.push(b);
        }
    }
    verts
}

/// Fan-triangulate a planar face from its boundary vertices.
fn tessellate_planar_face(
    face_vertex_ids: &[usize],
    vtx_node: &HashMap<usize, u64>,
    mesh: &mut Mesh,
    next_elem: &mut u64,
) {
    if face_vertex_ids.len() < 3 {
        return;
    }

    // Try to emit a quad if exactly 4 vertices
    if face_vertex_ids.len() == 4 {
        let nids: Vec<u64> = face_vertex_ids
            .iter()
            .filter_map(|vid| vtx_node.get(vid).copied())
            .collect();
        if nids.len() == 4 {
            mesh.add_element(Element::new(*next_elem, ElementType::Quad4, nids));
            *next_elem += 1;
            return;
        }
    }

    // Fan triangulation from first vertex
    let root = match vtx_node.get(&face_vertex_ids[0]) {
        Some(&nid) => nid,
        None => return,
    };
    for i in 1..(face_vertex_ids.len() - 1) {
        let n1 = vtx_node.get(&face_vertex_ids[i]).copied();
        let n2 = vtx_node.get(&face_vertex_ids[i + 1]).copied();
        if let (Some(a), Some(b)) = (n1, n2) {
            mesh.add_element(Element::new(
                *next_elem,
                ElementType::Triangle3,
                vec![root, a, b],
            ));
            *next_elem += 1;
        }
    }
}

/// Tessellate a curved face by sampling the surface on a parametric (u,v) grid.
fn tessellate_curved_face(
    shape: &Shape,
    face: &crate::shape::CadFace,
    face_vertex_ids: &[usize],
    deflection: f64,
    mesh: &mut Mesh,
    next_node: &mut u64,
    next_elem: &mut u64,
) {
    // Determine (u,v) range and grid resolution from the surface type.
    let (u_range, v_range, n_u, n_v) =
        parametric_grid_params(&face.surface, face_vertex_ids.len(), deflection);

    // Create grid nodes
    let mut grid: Vec<Vec<u64>> = Vec::with_capacity(n_v + 1);
    for j in 0..=n_v {
        let v = v_range.0 + (v_range.1 - v_range.0) * (j as f64) / (n_v as f64);
        let mut row = Vec::with_capacity(n_u + 1);
        for i in 0..=n_u {
            let u = u_range.0 + (u_range.1 - u_range.0) * (i as f64) / (n_u as f64);
            let p = face.surface.point_at(u, v);
            let nid = *next_node;
            *next_node += 1;
            mesh.add_node(Node::new(nid, p.x, p.y, p.z));
            row.push(nid);
        }
        grid.push(row);
    }

    // Emit triangles from the grid (2 triangles per quad cell)
    for j in 0..n_v {
        for i in 0..n_u {
            let n00 = grid[j][i];
            let n10 = grid[j][i + 1];
            let n01 = grid[j + 1][i];
            let n11 = grid[j + 1][i + 1];

            mesh.add_element(Element::new(
                *next_elem,
                ElementType::Triangle3,
                vec![n00, n10, n01],
            ));
            *next_elem += 1;
            mesh.add_element(Element::new(
                *next_elem,
                ElementType::Triangle3,
                vec![n10, n11, n01],
            ));
            *next_elem += 1;
        }
    }

    let _ = shape; // used only for vertex lookup if needed later
}

/// Determine (u_range, v_range, n_u, n_v) for a parametric grid based on surface type.
fn parametric_grid_params(
    surface: &Surface,
    _boundary_vertex_count: usize,
    deflection: f64,
) -> ((f64, f64), (f64, f64), usize, usize) {
    use std::f64::consts::PI;
    let base_n = (1.0 / deflection.max(0.01)).ceil() as usize;
    let base_n = base_n.clamp(4, 128);

    match surface {
        Surface::Plane { .. } => {
            // Planar faces should have been handled separately; fall back to unit square.
            ((0.0, 1.0), (0.0, 1.0), 2, 2)
        }
        Surface::Cylinder { .. } => ((0.0, 2.0 * PI), (0.0, 1.0), base_n, 2),
        Surface::Sphere { .. } => ((0.0, 2.0 * PI), (0.0, PI), base_n, base_n / 2),
        Surface::Cone { .. } => ((0.0, 2.0 * PI), (0.0, 1.0), base_n, 2),
        Surface::Torus { .. } => ((0.0, 2.0 * PI), (0.0, 2.0 * PI), base_n, base_n),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitive::make_box;
    use nalgebra::Point3;

    #[test]
    fn tessellate_box_produces_valid_mesh() {
        let b = make_box(1.0, 2.0, 3.0);
        let m = tessellate(&b, 0.1);
        assert!(m.node_count() >= 8, "box mesh should have at least 8 nodes");
        assert!(m.element_count() >= 6, "box mesh should have at least 6 elements");

        let (min, max) = m.bounding_box().expect("mesh should have bounding box");
        assert!((min.x - 0.0).abs() < 1e-12);
        assert!((min.y - 0.0).abs() < 1e-12);
        assert!((min.z - 0.0).abs() < 1e-12);
        assert!((max.x - 1.0).abs() < 1e-12);
        assert!((max.y - 2.0).abs() < 1e-12);
        assert!((max.z - 3.0).abs() < 1e-12);
    }

    #[test]
    fn tessellate_box_all_2d_elements() {
        let b = make_box(1.0, 1.0, 1.0);
        let m = tessellate(&b, 0.1);
        for e in &m.elements {
            assert_eq!(e.dimension(), 2, "box tessellation should produce 2D elements");
        }
    }

    #[test]
    fn tessellate_sphere_produces_mesh() {
        let sp = crate::primitive::make_sphere(Point3::origin(), 1.0, 8, 6);
        let m = tessellate(&sp, 0.1);
        assert!(m.node_count() > 0, "sphere mesh should have nodes");
        assert!(m.element_count() > 0, "sphere mesh should have elements");
    }
}
