use nalgebra::{Point3, Rotation3, Unit, Vector3};

use crate::shape::Shape;

/// Translate every vertex of the shape by `offset`, returning a new shape.
pub fn translate(shape: &Shape, offset: Vector3<f64>) -> Shape {
    let mut out = shape.clone();
    for v in &mut out.vertices {
        v.point += offset;
    }
    // Update curve origins in edges
    for e in &mut out.edges {
        match &mut e.curve {
            crate::geom::Curve::Line { origin, .. } => *origin += offset,
            crate::geom::Curve::Circle { center, .. } => *center += offset,
            crate::geom::Curve::Ellipse { center, .. } => *center += offset,
        }
    }
    // Update surface origins in faces
    for f in &mut out.faces {
        translate_surface(&mut f.surface, &offset);
    }
    out
}

/// Rotate every vertex of the shape about `axis` through the origin by `angle` radians,
/// returning a new shape.
pub fn rotate(shape: &Shape, axis: Vector3<f64>, angle: f64) -> Shape {
    let rot = Rotation3::from_axis_angle(&Unit::new_normalize(axis), angle);
    let mut out = shape.clone();
    for v in &mut out.vertices {
        v.point = Point3::from(rot * v.point.coords);
    }
    for e in &mut out.edges {
        match &mut e.curve {
            crate::geom::Curve::Line { origin, dir } => {
                *origin = Point3::from(rot * origin.coords);
                *dir = rot * *dir;
            }
            crate::geom::Curve::Circle {
                center,
                normal,
                radius: _,
            } => {
                *center = Point3::from(rot * center.coords);
                *normal = rot * *normal;
            }
            crate::geom::Curve::Ellipse {
                center,
                u_axis,
                v_axis,
            } => {
                *center = Point3::from(rot * center.coords);
                *u_axis = rot * *u_axis;
                *v_axis = rot * *v_axis;
            }
        }
    }
    for f in &mut out.faces {
        rotate_surface(&mut f.surface, &rot);
    }
    out
}

/// Scale every vertex of the shape by `(sx, sy, sz)` relative to the origin,
/// returning a new shape.
///
/// For uniform scaling `sx == sy == sz` analytical surfaces remain valid.
/// Non-uniform scaling may distort curved surfaces, but vertex positions will
/// be correct.
pub fn scale(shape: &Shape, sx: f64, sy: f64, sz: f64) -> Shape {
    let mut out = shape.clone();
    for v in &mut out.vertices {
        v.point.x *= sx;
        v.point.y *= sy;
        v.point.z *= sz;
    }
    for e in &mut out.edges {
        match &mut e.curve {
            crate::geom::Curve::Line { origin, dir } => {
                origin.x *= sx;
                origin.y *= sy;
                origin.z *= sz;
                dir.x *= sx;
                dir.y *= sy;
                dir.z *= sz;
            }
            crate::geom::Curve::Circle {
                center, radius, ..
            } => {
                center.x *= sx;
                center.y *= sy;
                center.z *= sz;
                // Approximate: use average scale factor for radius
                *radius *= (sx + sy + sz) / 3.0;
            }
            crate::geom::Curve::Ellipse {
                center,
                u_axis,
                v_axis,
            } => {
                center.x *= sx;
                center.y *= sy;
                center.z *= sz;
                u_axis.x *= sx;
                u_axis.y *= sy;
                u_axis.z *= sz;
                v_axis.x *= sx;
                v_axis.y *= sy;
                v_axis.z *= sz;
            }
        }
    }
    for f in &mut out.faces {
        scale_surface(&mut f.surface, sx, sy, sz);
    }
    out
}

// ---- internal helpers ----

fn translate_surface(surf: &mut crate::geom::Surface, offset: &Vector3<f64>) {
    match surf {
        crate::geom::Surface::Plane { origin, .. } => *origin += offset,
        crate::geom::Surface::Cylinder { origin, .. } => *origin += offset,
        crate::geom::Surface::Sphere { center, .. } => *center += offset,
        crate::geom::Surface::Cone { apex, .. } => *apex += offset,
        crate::geom::Surface::Torus { center, .. } => *center += offset,
    }
}

fn rotate_surface(surf: &mut crate::geom::Surface, rot: &Rotation3<f64>) {
    match surf {
        crate::geom::Surface::Plane { origin, normal } => {
            *origin = Point3::from(rot * origin.coords);
            *normal = rot * *normal;
        }
        crate::geom::Surface::Cylinder {
            origin,
            axis,
            radius: _,
        } => {
            *origin = Point3::from(rot * origin.coords);
            *axis = rot * *axis;
        }
        crate::geom::Surface::Sphere { center, .. } => {
            *center = Point3::from(rot * center.coords);
        }
        crate::geom::Surface::Cone { apex, axis, .. } => {
            *apex = Point3::from(rot * apex.coords);
            *axis = rot * *axis;
        }
        crate::geom::Surface::Torus { center, axis, .. } => {
            *center = Point3::from(rot * center.coords);
            *axis = rot * *axis;
        }
    }
}

fn scale_surface(surf: &mut crate::geom::Surface, sx: f64, sy: f64, sz: f64) {
    match surf {
        crate::geom::Surface::Plane { origin, normal } => {
            origin.x *= sx;
            origin.y *= sy;
            origin.z *= sz;
            // Normal doesn't scale the same way, but we keep it approximate
            normal.x *= sx;
            normal.y *= sy;
            normal.z *= sz;
            *normal = normal.normalize();
        }
        crate::geom::Surface::Cylinder {
            origin,
            axis,
            radius,
        } => {
            origin.x *= sx;
            origin.y *= sy;
            origin.z *= sz;
            axis.x *= sx;
            axis.y *= sy;
            axis.z *= sz;
            *radius *= (sx + sy + sz) / 3.0;
        }
        crate::geom::Surface::Sphere { center, radius } => {
            center.x *= sx;
            center.y *= sy;
            center.z *= sz;
            *radius *= (sx + sy + sz) / 3.0;
        }
        crate::geom::Surface::Cone { apex, axis, .. } => {
            apex.x *= sx;
            apex.y *= sy;
            apex.z *= sz;
            axis.x *= sx;
            axis.y *= sy;
            axis.z *= sz;
        }
        crate::geom::Surface::Torus {
            center,
            axis,
            major_radius,
            minor_radius,
        } => {
            center.x *= sx;
            center.y *= sy;
            center.z *= sz;
            axis.x *= sx;
            axis.y *= sy;
            axis.z *= sz;
            let avg = (sx + sy + sz) / 3.0;
            *major_radius *= avg;
            *minor_radius *= avg;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitive::make_box;
    use std::f64::consts::FRAC_PI_2;

    #[test]
    fn translate_shifts_vertices() {
        let b = make_box(1.0, 1.0, 1.0);
        let t = translate(&b, Vector3::new(10.0, 0.0, 0.0));
        for v in &t.vertices {
            assert!(v.point.x >= 10.0 - 1e-12);
        }
    }

    #[test]
    fn scale_doubles_size() {
        let b = make_box(1.0, 1.0, 1.0);
        let t = scale(&b, 2.0, 2.0, 2.0);
        let max_x = t.vertices.iter().map(|v| v.point.x).fold(0.0_f64, f64::max);
        assert!((max_x - 2.0).abs() < 1e-12);
    }

    #[test]
    fn rotate_90_about_z() {
        let b = make_box(1.0, 0.0, 0.0);
        // Degenerate box but tests rotation
        let r = rotate(&b, Vector3::z(), FRAC_PI_2);
        // After 90° about Z, x-axis maps to y-axis
        let max_y = r.vertices.iter().map(|v| v.point.y).fold(f64::MIN, f64::max);
        assert!(max_y >= 1.0 - 1e-9);
    }
}
