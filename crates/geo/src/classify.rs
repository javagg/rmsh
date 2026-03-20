//! Topology classification following gmsh's `classifyFaces` approach.
//!
//! Algorithm:
//! 1. Extract boundary faces from volume elements (or use 2D elements directly).
//! 2. Build face adjacency graph via shared edges.
//! 3. Compute dihedral angle between adjacent faces.
//! 4. Flood-fill faces into topological surfaces: adjacent faces whose dihedral
//!    angle is below the threshold belong to the same TopoFace.
//! 5. Edges between different TopoFaces become TopoEdges.
//! 6. Vertices where 3+ TopoEdges meet become TopoVertices.
//! 7. Volume elements grouped by connectivity form TopoVolumes.

use std::collections::{HashMap, HashSet, VecDeque};

use rmsh_model::{Mesh, Topology, TopoEdge, TopoFace, TopoVertex, TopoVolume};

/// A mesh face — a boundary polygon with its normal.
#[derive(Debug, Clone)]
struct MeshFace {
    /// Original node IDs in winding order.
    nodes: Vec<u64>,
    /// Face normal (unit vector).
    normal: [f64; 3],
}

/// Classify the mesh into a B-Rep-style topology.
///
/// Handles both pure-surface meshes (only 2D elements) and volume meshes
/// (extracts boundary faces from 3D elements).
pub fn classify(mesh: &Mesh, angle_threshold_deg: f64) -> Topology {
    let threshold_rad = angle_threshold_deg.to_radians();

    // Step 1: Collect boundary faces
    let faces = collect_boundary_faces(mesh);
    if faces.is_empty() {
        return Topology {
            angle_threshold_deg,
            ..Default::default()
        };
    }

    // Step 2: Build edge → face adjacency
    // An "edge" is a pair of node IDs (sorted).
    let mut edge_to_faces: HashMap<(u64, u64), Vec<usize>> = HashMap::new();
    for (fi, face) in faces.iter().enumerate() {
        let n = face.nodes.len();
        for i in 0..n {
            let a = face.nodes[i];
            let b = face.nodes[(i + 1) % n];
            let edge = if a < b { (a, b) } else { (b, a) };
            edge_to_faces.entry(edge).or_default().push(fi);
        }
    }

    // Build face-face adjacency with the shared edge
    // adj[fi] = vec of (neighbor_face_index, shared_edge)
    let mut adj: Vec<Vec<(usize, (u64, u64))>> = vec![Vec::new(); faces.len()];
    for (&edge, face_indices) in &edge_to_faces {
        for i in 0..face_indices.len() {
            for j in (i + 1)..face_indices.len() {
                let fi = face_indices[i];
                let fj = face_indices[j];
                adj[fi].push((fj, edge));
                adj[fj].push((fi, edge));
            }
        }
    }

    // Step 3 & 4: Flood-fill faces into TopoFaces using dihedral angle
    let mut face_to_topo: Vec<Option<usize>> = vec![None; faces.len()];
    let mut topo_faces: Vec<TopoFace> = Vec::new();

    for start in 0..faces.len() {
        if face_to_topo[start].is_some() {
            continue;
        }
        let topo_id = topo_faces.len();
        let mut queue = VecDeque::new();
        queue.push_back(start);
        face_to_topo[start] = Some(topo_id);

        let mut mesh_face_nodes: Vec<Vec<u64>> = Vec::new();

        while let Some(fi) = queue.pop_front() {
            mesh_face_nodes.push(faces[fi].nodes.clone());

            for &(nj, _edge) in &adj[fi] {
                if face_to_topo[nj].is_some() {
                    continue;
                }
                let angle = dihedral_angle(&faces[fi].normal, &faces[nj].normal);
                if angle < threshold_rad {
                    face_to_topo[nj] = Some(topo_id);
                    queue.push_back(nj);
                }
            }
        }

        topo_faces.push(TopoFace {
            id: topo_id,
            edge_ids: Vec::new(), // filled later
            mesh_faces: mesh_face_nodes,
        });
    }

    // Step 5: Identify TopoEdges — edges shared by faces belonging to different TopoFaces,
    // or boundary edges (only one face).
    let mut topo_edge_map: HashMap<(u64, u64), usize> = HashMap::new();
    let mut topo_edges: Vec<TopoEdge> = Vec::new();
    // Track which topo faces each topo edge borders.
    let mut edge_topo_faces: HashMap<usize, HashSet<usize>> = HashMap::new();

    for (&edge, face_indices) in &edge_to_faces {
        let topo_ids: HashSet<usize> = face_indices
            .iter()
            .filter_map(|&fi| face_to_topo[fi])
            .collect();

        let is_boundary = face_indices.len() == 1;
        let is_sharp = topo_ids.len() > 1;

        if is_boundary || is_sharp {
            let eid = topo_edges.len();
            topo_edge_map.insert(edge, eid);
            topo_edges.push(TopoEdge {
                id: eid,
                vertex_ids: [None, None], // filled later
                node_ids: vec![edge.0, edge.1],
            });
            edge_topo_faces.insert(eid, topo_ids);
        }
    }

    // Assign edge IDs to their TopoFaces
    for (&eid, topo_ids) in &edge_topo_faces {
        for &tid in topo_ids {
            if !topo_faces[tid].edge_ids.contains(&eid) {
                topo_faces[tid].edge_ids.push(eid);
            }
        }
    }

    // Step 6: Chain topo-edges and identify TopoVertices.
    // A TopoVertex is a node where 3+ topo edges meet (or 1 for dangling, 2 for corners).
    let mut node_to_edges: HashMap<u64, Vec<usize>> = HashMap::new();
    for (eid, te) in topo_edges.iter().enumerate() {
        for &nid in &te.node_ids {
            node_to_edges.entry(nid).or_default().push(eid);
        }
    }

    let mut topo_vertices: Vec<TopoVertex> = Vec::new();
    let mut node_to_vertex: HashMap<u64, usize> = HashMap::new();

    for (&nid, eids) in &node_to_edges {
        // A vertex where != 2 edges meet, or where it's a real corner
        if eids.len() != 2 {
            let vid = topo_vertices.len();
            topo_vertices.push(TopoVertex { id: vid, node_id: nid });
            node_to_vertex.insert(nid, vid);
        }
    }

    // Assign vertex IDs to TopoEdge endpoints
    for te in &mut topo_edges {
        let n0 = te.node_ids[0];
        let n1 = *te.node_ids.last().unwrap();
        te.vertex_ids[0] = node_to_vertex.get(&n0).copied();
        te.vertex_ids[1] = node_to_vertex.get(&n1).copied();
    }

    // Step 7: Group volume elements into TopoVolumes by connectivity
    let topo_volumes = classify_volumes(mesh);

    Topology {
        vertices: topo_vertices,
        edges: topo_edges,
        faces: topo_faces,
        volumes: topo_volumes,
        angle_threshold_deg,
    }
}

/// Collect boundary faces from a mesh.
/// For 3D meshes: faces of volume elements that appear only once (boundary).
/// For 2D meshes: all surface elements directly.
fn collect_boundary_faces(mesh: &Mesh) -> Vec<MeshFace> {
    let vol_elements = mesh.elements_by_dimension(3);
    let surf_elements = mesh.elements_by_dimension(2);

    let mut faces = Vec::new();

    if !vol_elements.is_empty() {
        // Count face occurrences — boundary faces appear once
        let mut face_count: HashMap<Vec<u64>, Vec<u64>> = HashMap::new();
        let mut face_seen: HashMap<Vec<u64>, usize> = HashMap::new();

        for elem in &vol_elements {
            let elem_faces = elem.etype.faces();
            for face_local in elem_faces {
                let face_nodes: Vec<u64> = face_local.iter().map(|&i| elem.node_ids[i]).collect();
                let mut sorted = face_nodes.clone();
                sorted.sort();
                *face_seen.entry(sorted.clone()).or_insert(0) += 1;
                face_count.entry(sorted).or_insert(face_nodes);
            }
        }

        for (sorted, count) in &face_seen {
            if *count == 1 {
                let nodes = face_count[sorted].clone();
                let normal = compute_face_normal(&nodes, mesh);
                faces.push(MeshFace {
                    nodes,
                    normal,
                });
            }
        }
    } else {
        // Pure surface mesh — use 2D elements directly
        for elem in &surf_elements {
            let nodes = elem.node_ids.clone();
            let normal = compute_face_normal(&nodes, mesh);
            faces.push(MeshFace { nodes, normal });
        }
    }

    faces
}

/// Compute the outward normal for a face defined by node IDs.
fn compute_face_normal(node_ids: &[u64], mesh: &Mesh) -> [f64; 3] {
    if node_ids.len() < 3 {
        return [0.0, 0.0, 1.0];
    }
    let p0 = &mesh.nodes[&node_ids[0]].position;
    let p1 = &mesh.nodes[&node_ids[1]].position;
    let p2 = &mesh.nodes[&node_ids[2]].position;

    let v1 = [p1.x - p0.x, p1.y - p0.y, p1.z - p0.z];
    let v2 = [p2.x - p0.x, p2.y - p0.y, p2.z - p0.z];

    let n = [
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0],
    ];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len > 1e-15 {
        [n[0] / len, n[1] / len, n[2] / len]
    } else {
        [0.0, 0.0, 1.0]
    }
}

/// Compute dihedral angle between two face normals (in radians).
/// Returns the angle between the normals (0 = coplanar, π = facing opposite).
fn dihedral_angle(n1: &[f64; 3], n2: &[f64; 3]) -> f64 {
    let dot = n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2];
    // Clamp to [-1, 1] for numerical safety
    dot.clamp(-1.0, 1.0).acos()
}

/// Classify volume elements into connected TopoVolumes using element adjacency.
fn classify_volumes(mesh: &Mesh) -> Vec<TopoVolume> {
    let vol_elements: Vec<_> = mesh
        .elements
        .iter()
        .filter(|e| e.dimension() == 3)
        .collect();

    if vol_elements.is_empty() {
        return Vec::new();
    }

    // Build node → element index mapping for adjacency
    let mut node_to_elems: HashMap<u64, Vec<usize>> = HashMap::new();
    for (i, elem) in vol_elements.iter().enumerate() {
        for &nid in &elem.node_ids {
            node_to_elems.entry(nid).or_default().push(i);
        }
    }

    // Flood-fill connected components
    let mut visited = vec![false; vol_elements.len()];
    let mut volumes = Vec::new();

    for start in 0..vol_elements.len() {
        if visited[start] {
            continue;
        }

        let vol_id = volumes.len();
        let mut queue = VecDeque::new();
        queue.push_back(start);
        visited[start] = true;
        let mut element_ids = Vec::new();

        while let Some(ei) = queue.pop_front() {
            element_ids.push(vol_elements[ei].id);

            // Find neighbors via shared nodes
            for &nid in &vol_elements[ei].node_ids {
                for &ni in &node_to_elems[&nid] {
                    if !visited[ni] {
                        visited[ni] = true;
                        queue.push_back(ni);
                    }
                }
            }
        }

        volumes.push(TopoVolume {
            id: vol_id,
            face_ids: Vec::new(), // could be filled by cross-referencing
            element_ids,
        });
    }

    volumes
}
