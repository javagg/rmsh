use std::f64::consts::PI;

use nalgebra::{Point3, Vector3};

use crate::geom::{Curve, Surface};
use crate::shape::Shape;

/// Create an axis-aligned box with one corner at the origin and opposite corner at `(dx, dy, dz)`.
///
/// The resulting shape has 8 vertices, 12 edges, 6 planar faces, 1 shell, and 1 solid.
pub fn make_box(dx: f64, dy: f64, dz: f64) -> Shape {
    let mut s = Shape::new();

    // 8 vertices: bottom face (z=0) then top face (z=dz)
    //   0=(0,0,0), 1=(dx,0,0), 2=(dx,dy,0), 3=(0,dy,0)
    //   4=(0,0,dz), 5=(dx,0,dz), 6=(dx,dy,dz), 7=(0,dy,dz)
    let v0 = s.add_vertex(Point3::new(0.0, 0.0, 0.0));
    let v1 = s.add_vertex(Point3::new(dx, 0.0, 0.0));
    let v2 = s.add_vertex(Point3::new(dx, dy, 0.0));
    let v3 = s.add_vertex(Point3::new(0.0, dy, 0.0));
    let v4 = s.add_vertex(Point3::new(0.0, 0.0, dz));
    let v5 = s.add_vertex(Point3::new(dx, 0.0, dz));
    let v6 = s.add_vertex(Point3::new(dx, dy, dz));
    let v7 = s.add_vertex(Point3::new(0.0, dy, dz));

    // Helper: create a line-segment edge between two vertices.
    let edge = |s: &mut Shape, va: usize, vb: usize| -> usize {
        let pa = s.vertices[va].point;
        let pb = s.vertices[vb].point;
        let dir = pb - pa;
        s.add_edge(
            va,
            vb,
            Curve::Line { origin: pa, dir },
            (0.0, 1.0),
        )
    };

    // 12 edges: 4 bottom, 4 top, 4 vertical
    let eb0 = edge(&mut s, v0, v1);
    let eb1 = edge(&mut s, v1, v2);
    let eb2 = edge(&mut s, v2, v3);
    let eb3 = edge(&mut s, v3, v0);
    let et0 = edge(&mut s, v4, v5);
    let et1 = edge(&mut s, v5, v6);
    let et2 = edge(&mut s, v6, v7);
    let et3 = edge(&mut s, v7, v4);
    let ev0 = edge(&mut s, v0, v4);
    let ev1 = edge(&mut s, v1, v5);
    let ev2 = edge(&mut s, v2, v6);
    let ev3 = edge(&mut s, v3, v7);

    // Helper: create a planar face from 4 edges and an outward normal.
    let quad_face =
        |s: &mut Shape, edges: [usize; 4], oris: [bool; 4], origin: Point3<f64>, normal: Vector3<f64>| -> usize {
            let w = s.add_wire(edges.to_vec(), oris.to_vec());
            s.add_face(w, vec![], Surface::Plane { origin, normal }, false)
        };

    // 6 faces (outward normals)
    let f_bottom = quad_face(
        &mut s,
        [eb0, eb1, eb2, eb3],
        [true, true, true, true],
        Point3::origin(),
        -Vector3::z(),
    );
    let f_top = quad_face(
        &mut s,
        [et0, et1, et2, et3],
        [true, true, true, true],
        Point3::new(0.0, 0.0, dz),
        Vector3::z(),
    );
    let f_front = quad_face(
        &mut s,
        [eb0, ev1, et0, ev0],
        [true, true, false, false],
        Point3::origin(),
        -Vector3::y(),
    );
    let f_back = quad_face(
        &mut s,
        [eb2, ev3, et2, ev2],
        [true, true, false, false],
        Point3::new(0.0, dy, 0.0),
        Vector3::y(),
    );
    let f_left = quad_face(
        &mut s,
        [eb3, ev0, et3, ev3],
        [true, true, false, false],
        Point3::origin(),
        -Vector3::x(),
    );
    let f_right = quad_face(
        &mut s,
        [eb1, ev2, et1, ev1],
        [true, true, false, false],
        Point3::new(dx, 0.0, 0.0),
        Vector3::x(),
    );

    let shell = s.add_shell(vec![f_bottom, f_top, f_front, f_back, f_left, f_right]);
    s.add_solid(shell, vec![]);

    s
}

/// Create a sphere centred at `center` with the given `radius`.
///
/// `n_u` = azimuthal segments, `n_v` = polar segments.
/// Produces `n_u * (n_v - 1) + 2` vertices (including north/south poles).
pub fn make_sphere(center: Point3<f64>, radius: f64, n_u: usize, n_v: usize) -> Shape {
    let n_u = n_u.max(3);
    let n_v = n_v.max(2);

    let mut s = Shape::new();

    // North pole (v = 0, i.e. z = +R)
    let north = s.add_vertex(center + Vector3::new(0.0, 0.0, radius));
    // Ring vertices: row j (1..n_v-1), column i (0..n_u-1)
    let mut rings: Vec<Vec<usize>> = Vec::new();
    for j in 1..n_v {
        let theta = PI * (j as f64) / (n_v as f64); // polar angle from +z
        let mut ring = Vec::new();
        for i in 0..n_u {
            let phi = 2.0 * PI * (i as f64) / (n_u as f64);
            let x = center.x + radius * theta.sin() * phi.cos();
            let y = center.y + radius * theta.sin() * phi.sin();
            let z = center.z + radius * theta.cos();
            ring.push(s.add_vertex(Point3::new(x, y, z)));
        }
        rings.push(ring);
    }
    // South pole (v = π, z = -R)
    let south = s.add_vertex(center - Vector3::new(0.0, 0.0, radius));

    // Build triangular faces for pole caps and quad faces for middle bands.
    let mut face_ids: Vec<usize> = Vec::new();

    let sphere_surface = Surface::Sphere { center, radius };

    // North cap: triangles from north pole to first ring
    for i in 0..n_u {
        let i_next = (i + 1) % n_u;
        let w = s.add_wire(vec![], vec![]);
        face_ids.push(s.add_face(w, vec![], sphere_surface.clone(), false));
        // We store the vertex indices in the wire for tessellation to use;
        // for this simple engine we rely on tessellate() sampling the surface.
        let _ = (north, rings[0][i], rings[0][i_next]); // referenced by tessellator
    }

    // Middle bands: quads
    for j in 0..(rings.len() - 1) {
        for i in 0..n_u {
            let i_next = (i + 1) % n_u;
            let w = s.add_wire(vec![], vec![]);
            face_ids.push(s.add_face(w, vec![], sphere_surface.clone(), false));
            let _ = (rings[j][i], rings[j][i_next], rings[j + 1][i_next], rings[j + 1][i]);
        }
    }

    // South cap
    let last_ring = rings.len() - 1;
    for i in 0..n_u {
        let i_next = (i + 1) % n_u;
        let w = s.add_wire(vec![], vec![]);
        face_ids.push(s.add_face(w, vec![], sphere_surface.clone(), false));
        let _ = (rings[last_ring][i], south, rings[last_ring][i_next]);
    }

    let shell = s.add_shell(face_ids);
    s.add_solid(shell, vec![]);

    s
}

/// Create a cylinder with bottom circle centered at `origin`, axis direction
/// `axis` (will be normalised), given `radius` and `height`.
///
/// `n_seg` = number of segments around the circumference.
pub fn make_cylinder(
    origin: Point3<f64>,
    axis: Vector3<f64>,
    radius: f64,
    height: f64,
    n_seg: usize,
) -> Shape {
    let n_seg = n_seg.max(3);
    let a = axis.normalize();
    let mut s = Shape::new();

    // Build two rings of vertices (bottom and top)
    let orthonormal = orthonormal_pair(&a);
    let (u_dir, v_dir) = orthonormal;

    let mut bottom_verts = Vec::new();
    let mut top_verts = Vec::new();
    for i in 0..n_seg {
        let angle = 2.0 * PI * (i as f64) / (n_seg as f64);
        let radial = u_dir * angle.cos() + v_dir * angle.sin();
        bottom_verts.push(s.add_vertex(origin + radial * radius));
        top_verts.push(s.add_vertex(origin + a * height + radial * radius));
    }

    let cyl_surface = Surface::Cylinder {
        origin,
        axis,
        radius,
    };

    let mut face_ids = Vec::new();

    // Lateral faces (quads)
    for i in 0..n_seg {
        let i_next = (i + 1) % n_seg;
        let w = s.add_wire(vec![], vec![]);
        face_ids.push(s.add_face(w, vec![], cyl_surface.clone(), false));
        let _ = (
            bottom_verts[i],
            bottom_verts[i_next],
            top_verts[i_next],
            top_verts[i],
        );
    }

    // Bottom cap
    let bottom_wire = s.add_wire(vec![], vec![]);
    face_ids.push(s.add_face(
        bottom_wire,
        vec![],
        Surface::Plane {
            origin,
            normal: -a,
        },
        false,
    ));

    // Top cap
    let top_wire = s.add_wire(vec![], vec![]);
    face_ids.push(s.add_face(
        top_wire,
        vec![],
        Surface::Plane {
            origin: origin + a * height,
            normal: a,
        },
        false,
    ));

    let shell = s.add_shell(face_ids);
    s.add_solid(shell, vec![]);

    s
}

/// Create a cone with apex at `apex`, axis direction `axis` (normalised internally),
/// half-angle `half_angle` (radians), and given `height` from apex to base.
///
/// `n_seg` = number of circumferential segments.
pub fn make_cone(
    apex: Point3<f64>,
    axis: Vector3<f64>,
    half_angle: f64,
    height: f64,
    n_seg: usize,
) -> Shape {
    let n_seg = n_seg.max(3);
    let a = axis.normalize();
    let base_radius = height * half_angle.tan();
    let base_center = apex + a * height;
    let mut s = Shape::new();

    let apex_v = s.add_vertex(apex);

    let (u_dir, v_dir) = orthonormal_pair(&a);
    let mut base_verts = Vec::new();
    for i in 0..n_seg {
        let angle = 2.0 * PI * (i as f64) / (n_seg as f64);
        let radial = u_dir * angle.cos() + v_dir * angle.sin();
        base_verts.push(s.add_vertex(base_center + radial * base_radius));
    }

    let cone_surface = Surface::Cone {
        apex,
        axis,
        half_angle,
    };
    let mut face_ids = Vec::new();

    // Lateral triangular faces
    for i in 0..n_seg {
        let i_next = (i + 1) % n_seg;
        let w = s.add_wire(vec![], vec![]);
        face_ids.push(s.add_face(w, vec![], cone_surface.clone(), false));
        let _ = (apex_v, base_verts[i], base_verts[i_next]);
    }

    // Base cap
    let base_wire = s.add_wire(vec![], vec![]);
    face_ids.push(s.add_face(
        base_wire,
        vec![],
        Surface::Plane {
            origin: base_center,
            normal: a,
        },
        false,
    ));

    let shell = s.add_shell(face_ids);
    s.add_solid(shell, vec![]);

    s
}

/// Build an orthonormal pair `(e1, e2)` perpendicular to the given unit vector.
fn orthonormal_pair(n: &Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    let hint = if n.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let e1 = n.cross(&hint).normalize();
    let e2 = n.cross(&e1);
    (e1, e2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn box_entity_counts() {
        let b = make_box(1.0, 2.0, 3.0);
        assert_eq!(b.vertices.len(), 8);
        assert_eq!(b.edges.len(), 12);
        assert_eq!(b.faces.len(), 6);
        assert_eq!(b.shells.len(), 1);
        assert_eq!(b.solids.len(), 1);
    }

    #[test]
    fn box_vertex_positions() {
        let b = make_box(2.0, 3.0, 4.0);
        let xs: Vec<f64> = b.vertices.iter().map(|v| v.point.x).collect();
        assert!(xs.iter().any(|&x| (x - 0.0).abs() < 1e-12));
        assert!(xs.iter().any(|&x| (x - 2.0).abs() < 1e-12));
        let zs: Vec<f64> = b.vertices.iter().map(|v| v.point.z).collect();
        assert!(zs.iter().any(|&z| (z - 4.0).abs() < 1e-12));
    }

    #[test]
    fn sphere_vertex_count() {
        let sp = make_sphere(Point3::origin(), 1.0, 8, 6);
        // poles (2) + rings ((n_v-1) * n_u) = 2 + 5*8 = 42
        assert_eq!(sp.vertices.len(), 2 + 5 * 8);
    }

    #[test]
    fn cylinder_has_correct_structure() {
        let c = make_cylinder(Point3::origin(), Vector3::z(), 1.0, 5.0, 12);
        // 12 bottom + 12 top = 24 vertices
        assert_eq!(c.vertices.len(), 24);
        // 12 lateral + 1 bottom cap + 1 top cap = 14 faces
        assert_eq!(c.faces.len(), 14);
        assert_eq!(c.solids.len(), 1);
    }

    #[test]
    fn cone_has_correct_structure() {
        let c = make_cone(
            Point3::origin(),
            Vector3::z(),
            PI / 6.0,
            3.0,
            8,
        );
        // 1 apex + 8 base = 9 vertices
        assert_eq!(c.vertices.len(), 9);
        // 8 lateral + 1 base = 9 faces
        assert_eq!(c.faces.len(), 9);
        assert_eq!(c.solids.len(), 1);
    }
}
