use nalgebra::Point3;

use crate::geom::{Curve, Surface};

/// A B-Rep vertex -- a point in 3D space.
#[derive(Debug, Clone)]
pub struct CadVertex {
    pub id: usize,
    pub point: Point3<f64>,
}

/// A B-Rep edge -- a curve segment bounded by two vertices.
#[derive(Debug, Clone)]
pub struct CadEdge {
    pub id: usize,
    pub start_vertex: usize,
    pub end_vertex: usize,
    pub curve: Curve,
    /// Parameter range `[t0, t1]` on the curve.
    pub t_range: (f64, f64),
}

/// A wire -- an ordered loop of edges forming a closed boundary.
#[derive(Debug, Clone)]
pub struct CadWire {
    pub id: usize,
    /// Indices into [`Shape::edges`].
    pub edge_ids: Vec<usize>,
    /// Whether each edge is traversed in its natural direction.
    pub orientations: Vec<bool>,
}

/// A B-Rep face -- a bounded portion of a surface.
#[derive(Debug, Clone)]
pub struct CadFace {
    pub id: usize,
    /// Index of the outer wire in [`Shape::wires`].
    pub outer_wire: usize,
    /// Indices of inner wires (holes) in [`Shape::wires`].
    pub inner_wires: Vec<usize>,
    /// Underlying analytical surface.
    pub surface: Surface,
    /// If `true`, the face normal is opposite to `surface.normal_at()`.
    pub reversed: bool,
}

/// A shell -- a connected set of faces forming a closed or open skin.
#[derive(Debug, Clone)]
pub struct CadShell {
    pub id: usize,
    /// Indices into [`Shape::faces`].
    pub face_ids: Vec<usize>,
}

/// A solid -- a watertight volume bounded by shells.
#[derive(Debug, Clone)]
pub struct CadSolid {
    pub id: usize,
    /// Index of the outer shell in [`Shape::shells`].
    pub outer_shell: usize,
    /// Indices of inner shells (voids) in [`Shape::shells`].
    pub inner_shells: Vec<usize>,
}

/// Top-level B-Rep shape container.
///
/// All entities are stored in arena-style `Vec`s. Cross-references use `usize`
/// indices into the corresponding `Vec`. Entity `.id` fields always equal their
/// index in the owning `Vec`.
#[derive(Debug, Clone, Default)]
pub struct Shape {
    pub vertices: Vec<CadVertex>,
    pub edges: Vec<CadEdge>,
    pub wires: Vec<CadWire>,
    pub faces: Vec<CadFace>,
    pub shells: Vec<CadShell>,
    pub solids: Vec<CadSolid>,
}

impl Shape {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a vertex and return its index.
    pub fn add_vertex(&mut self, point: Point3<f64>) -> usize {
        let id = self.vertices.len();
        self.vertices.push(CadVertex { id, point });
        id
    }

    /// Add an edge and return its index.
    pub fn add_edge(
        &mut self,
        start_vertex: usize,
        end_vertex: usize,
        curve: Curve,
        t_range: (f64, f64),
    ) -> usize {
        let id = self.edges.len();
        self.edges.push(CadEdge {
            id,
            start_vertex,
            end_vertex,
            curve,
            t_range,
        });
        id
    }

    /// Add a wire (edge loop) and return its index.
    pub fn add_wire(&mut self, edge_ids: Vec<usize>, orientations: Vec<bool>) -> usize {
        let id = self.wires.len();
        self.wires.push(CadWire {
            id,
            edge_ids,
            orientations,
        });
        id
    }

    /// Add a face and return its index.
    pub fn add_face(
        &mut self,
        outer_wire: usize,
        inner_wires: Vec<usize>,
        surface: Surface,
        reversed: bool,
    ) -> usize {
        let id = self.faces.len();
        self.faces.push(CadFace {
            id,
            outer_wire,
            inner_wires,
            surface,
            reversed,
        });
        id
    }

    /// Add a shell and return its index.
    pub fn add_shell(&mut self, face_ids: Vec<usize>) -> usize {
        let id = self.shells.len();
        self.shells.push(CadShell { id, face_ids });
        id
    }

    /// Add a solid and return its index.
    pub fn add_solid(&mut self, outer_shell: usize, inner_shells: Vec<usize>) -> usize {
        let id = self.solids.len();
        self.solids.push(CadSolid {
            id,
            outer_shell,
            inner_shells,
        });
        id
    }

    /// Total entity count across all dimensions.
    pub fn entity_count(&self) -> usize {
        self.vertices.len()
            + self.edges.len()
            + self.wires.len()
            + self.faces.len()
            + self.shells.len()
            + self.solids.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shape_add_vertex() {
        let mut s = Shape::new();
        let v = s.add_vertex(Point3::new(1.0, 2.0, 3.0));
        assert_eq!(v, 0);
        assert_eq!(s.vertices.len(), 1);
        assert_eq!(s.vertices[0].id, 0);
    }
}
