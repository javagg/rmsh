use serde::{Deserialize, Serialize};

/// A topological vertex — a corner point where 3+ edges meet.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopoVertex {
    pub id: usize,
    /// Mesh node ID at this vertex.
    pub node_id: u64,
}

/// A topological edge — a curve along a sharp feature boundary.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopoEdge {
    pub id: usize,
    /// Start and end topo vertex IDs. `None` for closed loops.
    pub vertex_ids: [Option<usize>; 2],
    /// Ordered mesh node IDs along this edge.
    pub node_ids: Vec<u64>,
}

/// A topological face — a smooth surface region bounded by sharp edges.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopoFace {
    pub id: usize,
    /// Bounding topo edge IDs.
    pub edge_ids: Vec<usize>,
    /// Mesh face polygons making up this face (each polygon is a list of node IDs).
    pub mesh_faces: Vec<Vec<u64>>,
}

/// A topological volume — a solid region of connected volume elements.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopoVolume {
    pub id: usize,
    /// Bounding topo face IDs.
    pub face_ids: Vec<usize>,
    /// Mesh element IDs belonging to this volume.
    pub element_ids: Vec<u64>,
}

/// Complete B-Rep-style topology of a mesh, built via dihedral-angle classification
/// following the approach used in gmsh's `classifyFaces`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Topology {
    pub vertices: Vec<TopoVertex>,
    pub edges: Vec<TopoEdge>,
    pub faces: Vec<TopoFace>,
    pub volumes: Vec<TopoVolume>,
    /// Dihedral angle threshold used for classification (degrees).
    pub angle_threshold_deg: f64,
}

impl Default for Topology {
    fn default() -> Self {
        Self {
            vertices: Vec::new(),
            edges: Vec::new(),
            faces: Vec::new(),
            volumes: Vec::new(),
            angle_threshold_deg: 40.0,
        }
    }
}

/// Selection of a topological entity for highlighting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TopoSelection {
    Volume(usize),
    Face(usize),
    Edge(usize),
    Vertex(usize),
}
