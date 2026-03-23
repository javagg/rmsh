pub mod element;
pub mod mesh;
pub mod node;
pub mod topology;

pub use element::{Element, ElementType};
pub use mesh::Mesh;
pub use node::Node;
pub use topology::{GModel, GVertex, GEdge, GFace, GRegion, GSelection};

// Backward compatibility aliases
pub use topology::{Topology, TopoVertex, TopoEdge, TopoFace, TopoVolume, TopoSelection};

/// 3D point type alias
pub type Point3 = nalgebra::Point3<f64>;
/// 3D vector type alias
pub type Vector3 = nalgebra::Vector3<f64>;
