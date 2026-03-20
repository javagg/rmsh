use serde::{Deserialize, Serialize};

/// Supported finite element types.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ElementType {
    /// 2-node line
    Line2,
    /// 3-node triangle
    Triangle3,
    /// 4-node quadrilateral
    Quad4,
    /// 4-node tetrahedron
    Tetrahedron4,
    /// 8-node hexahedron
    Hexahedron8,
    /// 6-node prism (wedge)
    Prism6,
    /// 5-node pyramid
    Pyramid5,
    /// 1-node point
    Point1,
    /// Unknown / unsupported type
    Unknown(i32),
}

impl ElementType {
    /// Convert from Gmsh element type ID (MSH v4 format).
    pub fn from_gmsh_type_id(id: i32) -> Self {
        match id {
            15 => ElementType::Point1,
            1 => ElementType::Line2,
            2 => ElementType::Triangle3,
            3 => ElementType::Quad4,
            4 => ElementType::Tetrahedron4,
            5 => ElementType::Hexahedron8,
            6 => ElementType::Prism6,
            7 => ElementType::Pyramid5,
            _ => ElementType::Unknown(id),
        }
    }

    /// Number of nodes for this element type.
    pub fn node_count(&self) -> usize {
        match self {
            ElementType::Point1 => 1,
            ElementType::Line2 => 2,
            ElementType::Triangle3 => 3,
            ElementType::Quad4 => 4,
            ElementType::Tetrahedron4 => 4,
            ElementType::Hexahedron8 => 8,
            ElementType::Prism6 => 6,
            ElementType::Pyramid5 => 5,
            ElementType::Unknown(_) => 0,
        }
    }

    /// Topological dimension of this element (0=point, 1=edge, 2=face, 3=volume).
    pub fn dimension(&self) -> u8 {
        match self {
            ElementType::Point1 => 0,
            ElementType::Line2 => 1,
            ElementType::Triangle3 | ElementType::Quad4 => 2,
            ElementType::Tetrahedron4 | ElementType::Hexahedron8 | ElementType::Prism6 | ElementType::Pyramid5 => 3,
            ElementType::Unknown(_) => 0,
        }
    }

    /// Return the faces of a volume element as arrays of local node indices.
    /// Each face is a slice of node indices (3 for triangular faces, 4 for quad faces).
    pub fn faces(&self) -> &[&[usize]] {
        match self {
            ElementType::Tetrahedron4 => &[
                &[0, 1, 2],
                &[0, 1, 3],
                &[1, 2, 3],
                &[0, 2, 3],
            ],
            ElementType::Hexahedron8 => &[
                &[0, 1, 2, 3],
                &[4, 5, 6, 7],
                &[0, 1, 5, 4],
                &[2, 3, 7, 6],
                &[0, 3, 7, 4],
                &[1, 2, 6, 5],
            ],
            ElementType::Prism6 => &[
                &[0, 1, 2],
                &[3, 4, 5],
                &[0, 1, 4, 3],
                &[1, 2, 5, 4],
                &[0, 2, 5, 3],
            ],
            ElementType::Pyramid5 => &[
                &[0, 1, 2, 3],
                &[0, 1, 4],
                &[1, 2, 4],
                &[2, 3, 4],
                &[0, 3, 4],
            ],
            _ => &[],
        }
    }

    /// Return the edges of an element as pairs of local node indices.
    pub fn edges(&self) -> &[[usize; 2]] {
        match self {
            ElementType::Line2 => &[[0, 1]],
            ElementType::Triangle3 => &[[0, 1], [1, 2], [2, 0]],
            ElementType::Quad4 => &[[0, 1], [1, 2], [2, 3], [3, 0]],
            ElementType::Tetrahedron4 => &[
                [0, 1], [1, 2], [2, 0],
                [0, 3], [1, 3], [2, 3],
            ],
            ElementType::Hexahedron8 => &[
                [0, 1], [1, 2], [2, 3], [3, 0],
                [4, 5], [5, 6], [6, 7], [7, 4],
                [0, 4], [1, 5], [2, 6], [3, 7],
            ],
            ElementType::Prism6 => &[
                [0, 1], [1, 2], [2, 0],
                [3, 4], [4, 5], [5, 3],
                [0, 3], [1, 4], [2, 5],
            ],
            ElementType::Pyramid5 => &[
                [0, 1], [1, 2], [2, 3], [3, 0],
                [0, 4], [1, 4], [2, 4], [3, 4],
            ],
            _ => &[],
        }
    }
}

/// A finite element consisting of a type and connectivity (node IDs).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Element {
    pub id: u64,
    pub etype: ElementType,
    /// Physical group tag
    pub physical_tag: Option<i32>,
    /// Node IDs forming this element (global IDs referencing `Node::id`).
    pub node_ids: Vec<u64>,
}

impl Element {
    pub fn new(id: u64, etype: ElementType, node_ids: Vec<u64>) -> Self {
        Self {
            id,
            etype,
            physical_tag: None,
            node_ids,
        }
    }

    pub fn dimension(&self) -> u8 {
        self.etype.dimension()
    }
}
