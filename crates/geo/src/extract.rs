use std::collections::{HashMap, HashSet};

use rmsh_model::Mesh;

/// Extracted surface data ready for rendering.
pub struct SurfaceData {
    /// Triangle vertices as [x, y, z] positions.
    pub positions: Vec<[f32; 3]>,
    /// Triangle normals.
    pub normals: Vec<[f32; 3]>,
    /// Triangle indices.
    pub indices: Vec<u32>,
}

/// Extracted wireframe data ready for rendering.
pub struct WireframeData {
    /// Line segment endpoints as [x, y, z].
    pub positions: Vec<[f32; 3]>,
    /// Line indices (pairs).
    pub indices: Vec<u32>,
}

/// Extracted point data for rendering nodes.
pub struct PointData {
    pub positions: Vec<[f32; 3]>,
}

/// Extract the boundary surface triangles from volume elements.
/// Returns triangulated surface faces with computed normals.
pub fn extract_surface(mesh: &Mesh) -> SurfaceData {
    let volume_elements = mesh.elements_by_dimension(3);

    // Count face occurrences to find boundary faces
    // A face shared by two volume elements is internal, otherwise it's a boundary face
    let mut face_count: HashMap<Vec<u64>, (Vec<u64>, usize)> = HashMap::new();

    for elem in &volume_elements {
        let faces = elem.etype.faces();
        for face_local in faces {
            let face_nodes: Vec<u64> = face_local.iter().map(|&i| elem.node_ids[i]).collect();
            let mut sorted = face_nodes.clone();
            sorted.sort();
            let entry = face_count.entry(sorted).or_insert((face_nodes.clone(), 0));
            entry.1 += 1;
        }
    }

    // Also include 2D surface elements directly
    let surface_elements = mesh.elements_by_dimension(2);

    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut indices = Vec::new();

    let get_pos = |node_id: u64| -> [f32; 3] {
        let node = &mesh.nodes[&node_id];
        [node.position.x as f32, node.position.y as f32, node.position.z as f32]
    };

    // Add boundary faces from volume elements
    for (_, (face_nodes, count)) in &face_count {
        if *count == 1 {
            add_face_triangles(&face_nodes, &get_pos, &mut positions, &mut normals, &mut indices);
        }
    }

    // Add surface elements
    for elem in &surface_elements {
        add_face_triangles(&elem.node_ids, &get_pos, &mut positions, &mut normals, &mut indices);
    }

    SurfaceData {
        positions,
        normals,
        indices,
    }
}

/// Extract wireframe edges from elements of specified dimensions.
pub fn extract_wireframe(mesh: &Mesh, include_dim: &[u8]) -> WireframeData {
    let mut positions = Vec::new();
    let mut indices = Vec::new();
    let mut seen_edges: HashSet<(u64, u64)> = HashSet::new();

    for elem in &mesh.elements {
        if !include_dim.contains(&elem.dimension()) {
            continue;
        }
        let edges = elem.etype.edges();
        for [i, j] in edges {
            let a = elem.node_ids[*i];
            let b = elem.node_ids[*j];
            let edge = if a < b { (a, b) } else { (b, a) };
            if seen_edges.insert(edge) {
                let na = &mesh.nodes[&a];
                let nb = &mesh.nodes[&b];
                let idx = positions.len() as u32;
                positions.push([na.position.x as f32, na.position.y as f32, na.position.z as f32]);
                positions.push([nb.position.x as f32, nb.position.y as f32, nb.position.z as f32]);
                indices.push(idx);
                indices.push(idx + 1);
            }
        }
    }

    WireframeData { positions, indices }
}

/// Extract all node positions.
pub fn extract_points(mesh: &Mesh) -> PointData {
    let positions = mesh
        .nodes
        .values()
        .map(|n| [n.position.x as f32, n.position.y as f32, n.position.z as f32])
        .collect();
    PointData { positions }
}

/// Triangulate a face (3 or 4 nodes) and append to output buffers.
fn add_face_triangles(
    face_nodes: &[u64],
    get_pos: &impl Fn(u64) -> [f32; 3],
    positions: &mut Vec<[f32; 3]>,
    normals: &mut Vec<[f32; 3]>,
    indices: &mut Vec<u32>,
) {
    if face_nodes.len() < 3 {
        return;
    }

    let p: Vec<[f32; 3]> = face_nodes.iter().map(|&id| get_pos(id)).collect();
    let normal = compute_normal(&p[0], &p[1], &p[2]);

    // First triangle
    let base = positions.len() as u32;
    for pos in &p {
        positions.push(*pos);
        normals.push(normal);
    }
    indices.extend_from_slice(&[base, base + 1, base + 2]);

    // If quad, second triangle
    if face_nodes.len() == 4 {
        indices.extend_from_slice(&[base, base + 2, base + 3]);
    }
}

fn compute_normal(a: &[f32; 3], b: &[f32; 3], c: &[f32; 3]) -> [f32; 3] {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let n = [
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    ];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len > 1e-10 {
        [n[0] / len, n[1] / len, n[2] / len]
    } else {
        [0.0, 0.0, 1.0]
    }
}
