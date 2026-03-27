//! rmsh-cad — A simple constructive solid geometry (CSG) CAD engine.
//!
//! Provides B-Rep shape representation with analytical curve/surface geometry,
//! parametric primitive builders, affine transforms, tessellation to [`rmsh_model::Mesh`],
//! and approximate mesh-level boolean operations.

pub mod boolean;
pub mod geom;
pub mod primitive;
pub mod shape;
pub mod tessellate;
pub mod transform;

pub use geom::{Curve, Surface};
pub use primitive::{make_box, make_cone, make_cylinder, make_sphere};
pub use shape::{CadEdge, CadFace, CadShell, CadSolid, CadVertex, CadWire, Shape};
pub use boolean::{boolean, boolean_difference, boolean_intersection, boolean_union, BooleanOp};
pub use tessellate::tessellate;
pub use transform::{rotate, scale, translate};
