pub use rmsh_io::{parse_msh, MshError};

pub mod triangulate2d;
pub use triangulate2d::{mesh_polygon, triangulate_points, MeshError, Polygon2D};

pub mod tetrahedralize3d;
pub use tetrahedralize3d::{tetrahedralize_closed_surface, Mesh3DError};

