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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ElementFamily {
    Point,
    Line,
    Triangle,
    Quadrilateral,
    Tetrahedron,
    Hexahedron,
    Prism,
    Pyramid,
}

fn family_from_gmsh_type_id(id: i32) -> Option<ElementFamily> {
    match id {
        // Point
        15 => Some(ElementFamily::Point),
        // Line (1st+high order)
        1 | 8 | 26 | 27 | 28 => Some(ElementFamily::Line),
        // Triangle (1st+high order)
        2 | 9 | 20 | 21 | 22 | 23 | 24 | 25 => Some(ElementFamily::Triangle),
        // Quadrilateral (1st+high order)
        3 | 10 | 16 | 36 | 37 | 38 | 47 | 48 | 49 | 50 | 51 => Some(ElementFamily::Quadrilateral),
        // Tetrahedron (1st+high order)
        4 | 11 | 29 | 30 | 31 => Some(ElementFamily::Tetrahedron),
        // Hexahedron (1st+high order)
        5 | 12 | 17 | 92 | 93 => Some(ElementFamily::Hexahedron),
        // Prism (1st+high order)
        6 | 13 | 18 | 90 | 91 => Some(ElementFamily::Prism),
        // Pyramid (1st+high order)
        7 | 14 | 19 | 118 | 119 => Some(ElementFamily::Pyramid),
        _ => None,
    }
}

fn gmsh_dimension_from_type_id(id: i32) -> u8 {
    match family_from_gmsh_type_id(id) {
        Some(ElementFamily::Point) => 0,
        Some(ElementFamily::Line) => 1,
        Some(ElementFamily::Triangle) | Some(ElementFamily::Quadrilateral) => 2,
        Some(ElementFamily::Tetrahedron)
        | Some(ElementFamily::Hexahedron)
        | Some(ElementFamily::Prism)
        | Some(ElementFamily::Pyramid) => 3,
        None => 0,
    }
}

const LINE2_EDGES: &[[usize; 2]] = &[[0, 1]];
const TRI3_EDGES: &[[usize; 2]] = &[[0, 1], [1, 2], [2, 0]];
const QUAD4_EDGES: &[[usize; 2]] = &[[0, 1], [1, 2], [2, 3], [3, 0]];
const TET4_EDGES: &[[usize; 2]] = &[[0, 1], [1, 2], [2, 0], [0, 3], [1, 3], [2, 3]];
const HEX8_EDGES: &[[usize; 2]] = &[
    [0, 1],
    [1, 2],
    [2, 3],
    [3, 0],
    [4, 5],
    [5, 6],
    [6, 7],
    [7, 4],
    [0, 4],
    [1, 5],
    [2, 6],
    [3, 7],
];
const PRISM6_EDGES: &[[usize; 2]] = &[
    [0, 1],
    [1, 2],
    [2, 0],
    [3, 4],
    [4, 5],
    [5, 3],
    [0, 3],
    [1, 4],
    [2, 5],
];
const PYRAMID5_EDGES: &[[usize; 2]] = &[
    [0, 1],
    [1, 2],
    [2, 3],
    [3, 0],
    [0, 4],
    [1, 4],
    [2, 4],
    [3, 4],
];

const TET4_FACES: &[&[usize]] = &[&[0, 1, 2], &[0, 1, 3], &[1, 2, 3], &[0, 2, 3]];
const HEX8_FACES: &[&[usize]] = &[
    &[0, 1, 2, 3],
    &[4, 5, 6, 7],
    &[0, 1, 5, 4],
    &[2, 3, 7, 6],
    &[0, 3, 7, 4],
    &[1, 2, 6, 5],
];
const PRISM6_FACES: &[&[usize]] = &[
    &[0, 1, 2],
    &[3, 4, 5],
    &[0, 1, 4, 3],
    &[1, 2, 5, 4],
    &[0, 2, 5, 3],
];
const PYRAMID5_FACES: &[&[usize]] = &[
    &[0, 1, 2, 3],
    &[0, 1, 4],
    &[1, 2, 4],
    &[2, 3, 4],
    &[0, 3, 4],
];

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
            ElementType::Unknown(id) => gmsh_dimension_from_type_id(*id),
        }
    }

    /// Return the faces of a volume element as arrays of local node indices.
    /// Each face is a slice of node indices (3 for triangular faces, 4 for quad faces).
    pub fn faces(&self) -> &[&[usize]] {
        match self {
            ElementType::Tetrahedron4 => TET4_FACES,
            ElementType::Hexahedron8 => HEX8_FACES,
            ElementType::Prism6 => PRISM6_FACES,
            ElementType::Pyramid5 => PYRAMID5_FACES,
            ElementType::Unknown(id) => match family_from_gmsh_type_id(*id) {
                Some(ElementFamily::Tetrahedron) => TET4_FACES,
                Some(ElementFamily::Hexahedron) => HEX8_FACES,
                Some(ElementFamily::Prism) => PRISM6_FACES,
                Some(ElementFamily::Pyramid) => PYRAMID5_FACES,
                _ => &[],
            },
            _ => &[],
        }
    }

    /// Return the edges of an element as pairs of local node indices.
    pub fn edges(&self) -> &[[usize; 2]] {
        match self {
            ElementType::Line2 => LINE2_EDGES,
            ElementType::Triangle3 => TRI3_EDGES,
            ElementType::Quad4 => QUAD4_EDGES,
            ElementType::Tetrahedron4 => TET4_EDGES,
            ElementType::Hexahedron8 => HEX8_EDGES,
            ElementType::Prism6 => PRISM6_EDGES,
            ElementType::Pyramid5 => PYRAMID5_EDGES,
            ElementType::Unknown(id) => match family_from_gmsh_type_id(*id) {
                Some(ElementFamily::Line) => LINE2_EDGES,
                Some(ElementFamily::Triangle) => TRI3_EDGES,
                Some(ElementFamily::Quadrilateral) => QUAD4_EDGES,
                Some(ElementFamily::Tetrahedron) => TET4_EDGES,
                Some(ElementFamily::Hexahedron) => HEX8_EDGES,
                Some(ElementFamily::Prism) => PRISM6_EDGES,
                Some(ElementFamily::Pyramid) => PYRAMID5_EDGES,
                _ => &[],
            },
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

#[cfg(test)]
mod tests {
    use super::ElementType;

    #[test]
    fn unknown_high_order_types_have_correct_dimension() {
        assert_eq!(ElementType::Unknown(8).dimension(), 1); // line3
        assert_eq!(ElementType::Unknown(9).dimension(), 2); // tri6
        assert_eq!(ElementType::Unknown(10).dimension(), 2); // quad9
        assert_eq!(ElementType::Unknown(11).dimension(), 3); // tet10
        assert_eq!(ElementType::Unknown(12).dimension(), 3); // hex27
        assert_eq!(ElementType::Unknown(13).dimension(), 3); // prism18
        assert_eq!(ElementType::Unknown(14).dimension(), 3); // pyramid14
    }

    #[test]
    fn unknown_volume_families_expose_canonical_faces_and_edges() {
        assert_eq!(ElementType::Unknown(11).faces().len(), 4);
        assert_eq!(ElementType::Unknown(11).edges().len(), 6);

        assert_eq!(ElementType::Unknown(12).faces().len(), 6);
        assert_eq!(ElementType::Unknown(12).edges().len(), 12);

        assert_eq!(ElementType::Unknown(13).faces().len(), 5);
        assert_eq!(ElementType::Unknown(13).edges().len(), 9);

        assert_eq!(ElementType::Unknown(14).faces().len(), 5);
        assert_eq!(ElementType::Unknown(14).edges().len(), 8);
    }
}
