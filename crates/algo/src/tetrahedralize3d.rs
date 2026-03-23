use std::collections::{BTreeMap, HashSet};

use rmsh_model::{Element, ElementType, Mesh, Node, Point3};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum Mesh3DError {
    #[error("3D meshing failed: {0}")]
    Generation(String),
}

/// Build a simple tetrahedral volume mesh from a closed boundary surface.
///
/// The method is intentionally simple:
/// - Build boundary polygons from either 3D element boundary faces or existing 2D elements.
/// - Triangulate polygons with a fan split.
/// - Add one interior node at the boundary centroid.
/// - Create one tetrahedron per boundary triangle: `(a, b, c, centroid)`.
///
/// This works best for star-shaped or convex-like closed surfaces.
pub fn tetrahedralize_closed_surface(mesh: &Mesh) -> Result<Mesh, Mesh3DError> {
    let boundary_polys = collect_boundary_polygons(mesh)?;
    if boundary_polys.is_empty() {
        return Err(Mesh3DError::Generation(
            "No boundary polygons found for 3D meshing".to_string(),
        ));
    }

    let mut boundary_tris: Vec<[u64; 3]> = Vec::new();
    for poly in &boundary_polys {
        if poly.len() < 3 {
            continue;
        }
        for i in 1..(poly.len() - 1) {
            boundary_tris.push([poly[0], poly[i], poly[i + 1]]);
        }
    }
    if boundary_tris.is_empty() {
        return Err(Mesh3DError::Generation(
            "Boundary triangulation produced no triangles".to_string(),
        ));
    }

    let mut boundary_nodes: HashSet<u64> = HashSet::new();
    for tri in &boundary_tris {
        boundary_nodes.insert(tri[0]);
        boundary_nodes.insert(tri[1]);
        boundary_nodes.insert(tri[2]);
    }

    let centroid = centroid_of_nodes(mesh, &boundary_nodes)?;
    let centroid_id = mesh
        .nodes
        .keys()
        .copied()
        .max()
        .unwrap_or(0)
        .saturating_add(1);

    let mut out = Mesh::new();
    for nid in &boundary_nodes {
        let node = mesh
            .nodes
            .get(nid)
            .ok_or_else(|| Mesh3DError::Generation(format!("Node {} missing in source mesh", nid)))?;
        out.add_node(node.clone());
    }
    out.add_node(Node::new(centroid_id, centroid.x, centroid.y, centroid.z));

    let mut eid: u64 = 1;
    let mut accepted = 0usize;
    for tri in &boundary_tris {
        let pa = out
            .nodes
            .get(&tri[0])
            .ok_or_else(|| Mesh3DError::Generation("Missing tetra node A".to_string()))?
            .position;
        let pb = out
            .nodes
            .get(&tri[1])
            .ok_or_else(|| Mesh3DError::Generation("Missing tetra node B".to_string()))?
            .position;
        let pc = out
            .nodes
            .get(&tri[2])
            .ok_or_else(|| Mesh3DError::Generation("Missing tetra node C".to_string()))?
            .position;

        let vol6 = tetra_signed_volume6(pa, pb, pc, centroid).abs();
        if vol6 < 1e-14 {
            continue;
        }

        out.add_element(Element::new(
            eid,
            ElementType::Tetrahedron4,
            vec![tri[0], tri[1], tri[2], centroid_id],
        ));
        eid += 1;
        accepted += 1;
    }

    if accepted == 0 {
        return Err(Mesh3DError::Generation(
            "Generated tetrahedra are degenerate".to_string(),
        ));
    }

    Ok(out)
}

fn collect_boundary_polygons(mesh: &Mesh) -> Result<Vec<Vec<u64>>, Mesh3DError> {
    let mut has_volume = false;
    let mut face_counts: BTreeMap<Vec<u64>, (usize, Vec<u64>)> = BTreeMap::new();

    for elem in &mesh.elements {
        if elem.dimension() != 3 {
            continue;
        }
        has_volume = true;

        let faces = elem.etype.faces();
        if faces.is_empty() {
            continue;
        }

        for local_face in faces {
            let mut face_nodes = Vec::with_capacity(local_face.len());
            for li in *local_face {
                let Some(nid) = elem.node_ids.get(*li) else {
                    return Err(Mesh3DError::Generation(format!(
                        "Element {} face connectivity is inconsistent",
                        elem.id
                    )));
                };
                face_nodes.push(*nid);
            }

            let mut key = face_nodes.clone();
            key.sort_unstable();
            let entry = face_counts.entry(key).or_insert((0usize, face_nodes));
            entry.0 += 1;
        }
    }

    if has_volume {
        let polys = face_counts
            .into_values()
            .filter_map(|(count, face)| (count == 1).then_some(face))
            .collect::<Vec<_>>();
        return Ok(polys);
    }

    let polys = mesh
        .elements
        .iter()
        .filter(|e| e.dimension() == 2 && e.node_ids.len() >= 3)
        .map(|e| e.node_ids.clone())
        .collect::<Vec<_>>();
    Ok(polys)
}

fn centroid_of_nodes(mesh: &Mesh, ids: &HashSet<u64>) -> Result<Point3, Mesh3DError> {
    if ids.is_empty() {
        return Err(Mesh3DError::Generation(
            "Cannot compute centroid from an empty node set".to_string(),
        ));
    }

    let mut sx = 0.0;
    let mut sy = 0.0;
    let mut sz = 0.0;
    let mut n = 0usize;
    for nid in ids {
        let node = mesh
            .nodes
            .get(nid)
            .ok_or_else(|| Mesh3DError::Generation(format!("Node {} not found", nid)))?;
        sx += node.position.x;
        sy += node.position.y;
        sz += node.position.z;
        n += 1;
    }

    let inv = 1.0 / n as f64;
    Ok(Point3::new(sx * inv, sy * inv, sz * inv))
}

fn tetra_signed_volume6(a: Point3, b: Point3, c: Point3, d: Point3) -> f64 {
    let ab = b - a;
    let ac = c - a;
    let ad = d - a;
    ab.cross(&ac).dot(&ad)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tetrahedralize_cube_surface() {
        let mut mesh = Mesh::new();
        mesh.add_node(Node::new(1, 0.0, 0.0, 0.0));
        mesh.add_node(Node::new(2, 1.0, 0.0, 0.0));
        mesh.add_node(Node::new(3, 1.0, 1.0, 0.0));
        mesh.add_node(Node::new(4, 0.0, 1.0, 0.0));
        mesh.add_node(Node::new(5, 0.0, 0.0, 1.0));
        mesh.add_node(Node::new(6, 1.0, 0.0, 1.0));
        mesh.add_node(Node::new(7, 1.0, 1.0, 1.0));
        mesh.add_node(Node::new(8, 0.0, 1.0, 1.0));

        // 6 quad faces
        mesh.add_element(Element::new(1, ElementType::Quad4, vec![1, 2, 3, 4]));
        mesh.add_element(Element::new(2, ElementType::Quad4, vec![5, 6, 7, 8]));
        mesh.add_element(Element::new(3, ElementType::Quad4, vec![1, 2, 6, 5]));
        mesh.add_element(Element::new(4, ElementType::Quad4, vec![2, 3, 7, 6]));
        mesh.add_element(Element::new(5, ElementType::Quad4, vec![3, 4, 8, 7]));
        mesh.add_element(Element::new(6, ElementType::Quad4, vec![4, 1, 5, 8]));

        let out = tetrahedralize_closed_surface(&mesh).expect("tetrahedralization should succeed");
        assert_eq!(out.elements_by_dimension(3).len(), 12);
        assert!(out.node_count() >= 9);
    }
}
