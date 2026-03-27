use nalgebra::{Point3, UnitVector3, Vector3};

/// An analytical curve in 3D space.
#[derive(Debug, Clone)]
pub enum Curve {
    /// Infinite line defined by a point and direction.
    Line {
        origin: Point3<f64>,
        dir: Vector3<f64>,
    },
    /// Circle in 3D, lying in the plane perpendicular to `normal`.
    Circle {
        center: Point3<f64>,
        normal: Vector3<f64>,
        radius: f64,
    },
    /// Ellipse in 3D with two semi-axes `u_axis` (major) and `v_axis` (minor).
    Ellipse {
        center: Point3<f64>,
        u_axis: Vector3<f64>,
        v_axis: Vector3<f64>,
    },
}

impl Curve {
    /// Evaluate the curve at parameter `t`.
    ///
    /// - **Line**: `origin + t * dir`
    /// - **Circle**: `center + R * cos(t) * u + R * sin(t) * v` where `(u,v)` is an
    ///   orthonormal basis in the circle plane, `t ∈ [0, 2π)`.
    /// - **Ellipse**: `center + cos(t) * u_axis + sin(t) * v_axis`, `t ∈ [0, 2π)`.
    pub fn point_at(&self, t: f64) -> Point3<f64> {
        match self {
            Curve::Line { origin, dir } => origin + dir * t,
            Curve::Circle {
                center,
                normal,
                radius,
            } => {
                let (u, v) = make_plane_basis(normal);
                center + (u * t.cos() + v * t.sin()) * *radius
            }
            Curve::Ellipse {
                center,
                u_axis,
                v_axis,
            } => center + u_axis * t.cos() + v_axis * t.sin(),
        }
    }
}

/// An analytical surface in 3D space.
#[derive(Debug, Clone)]
pub enum Surface {
    /// Infinite plane.
    Plane {
        origin: Point3<f64>,
        normal: Vector3<f64>,
    },
    /// Cylindrical surface of infinite extent along `axis`.
    Cylinder {
        origin: Point3<f64>,
        axis: Vector3<f64>,
        radius: f64,
    },
    /// Sphere.
    Sphere {
        center: Point3<f64>,
        radius: f64,
    },
    /// Conical surface.
    Cone {
        apex: Point3<f64>,
        axis: Vector3<f64>,
        half_angle: f64,
    },
    /// Torus.
    Torus {
        center: Point3<f64>,
        axis: Vector3<f64>,
        major_radius: f64,
        minor_radius: f64,
    },
}

impl Surface {
    /// Evaluate the surface at parameters `(u, v)`.
    ///
    /// Parameterisation conventions:
    /// - **Plane**: `origin + u * e1 + v * e2` where `(e1, e2)` span the plane.
    /// - **Cylinder**: `origin + axis * v + R * (cos(u) * e1 + sin(u) * e2)`, `u ∈ [0,2π)`.
    /// - **Sphere**: standard spherical with `u = azimuth ∈ [0,2π)`, `v = polar ∈ [0,π]`.
    /// - **Cone**: `apex + (v * tan(half_angle)) * (cos(u)*e1 + sin(u)*e2) + v * axis`, `u∈[0,2π)`, `v≥0`.
    /// - **Torus**: `center + (R + r cos v)(cos(u) e1 + sin(u) e2) + r sin(v) axis`.
    pub fn point_at(&self, u: f64, v: f64) -> Point3<f64> {
        match self {
            Surface::Plane { origin, normal } => {
                let (e1, e2) = make_plane_basis(normal);
                origin + e1 * u + e2 * v
            }
            Surface::Cylinder {
                origin,
                axis,
                radius,
            } => {
                let (e1, e2) = make_plane_basis(axis);
                origin + axis.normalize() * v + (e1 * u.cos() + e2 * u.sin()) * *radius
            }
            Surface::Sphere { center, radius } => {
                let x = radius * v.sin() * u.cos();
                let y = radius * v.sin() * u.sin();
                let z = radius * v.cos();
                Point3::new(center.x + x, center.y + y, center.z + z)
            }
            Surface::Cone {
                apex,
                axis,
                half_angle,
            } => {
                let (e1, e2) = make_plane_basis(axis);
                let r = v * half_angle.tan();
                apex + axis.normalize() * v + (e1 * u.cos() + e2 * u.sin()) * r
            }
            Surface::Torus {
                center,
                axis,
                major_radius,
                minor_radius,
            } => {
                let (e1, e2) = make_plane_basis(axis);
                let a = axis.normalize();
                let rr = major_radius + minor_radius * v.cos();
                let coords =
                    center.coords + e1 * rr * u.cos() + e2 * rr * u.sin() + a * *minor_radius * v.sin();
                Point3::from(coords)
            }
        }
    }

    /// Outward-pointing unit normal at `(u, v)`.
    pub fn normal_at(&self, u: f64, v: f64) -> Vector3<f64> {
        match self {
            Surface::Plane { normal, .. } => normal.normalize(),
            Surface::Cylinder { axis, .. } => {
                let (e1, e2) = make_plane_basis(axis);
                (e1 * u.cos() + e2 * u.sin()).normalize()
            }
            Surface::Sphere { .. } => {
                let n = Vector3::new(v.sin() * u.cos(), v.sin() * u.sin(), v.cos());
                n.normalize()
            }
            Surface::Cone { axis, half_angle, .. } => {
                let (e1, e2) = make_plane_basis(axis);
                let a = axis.normalize();
                let radial = (e1 * u.cos() + e2 * u.sin()).normalize();
                (radial * half_angle.cos() - a * half_angle.sin()).normalize()
            }
            Surface::Torus { axis, .. } => {
                let (e1, e2) = make_plane_basis(axis);
                let a = axis.normalize();
                let radial = (e1 * u.cos() + e2 * u.sin()).normalize();
                (radial * v.cos() + a * v.sin()).normalize()
            }
        }
    }
}

/// Build an orthonormal basis `(e1, e2)` in the plane perpendicular to `n`.
fn make_plane_basis(n: &Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    let unit = UnitVector3::try_new(*n, 1e-12).unwrap_or(UnitVector3::new_normalize(Vector3::z()));
    let arbitrary = if unit.dot(&Vector3::x()).abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let e1 = unit.cross(&arbitrary).normalize();
    let e2 = unit.cross(&e1).normalize();
    (e1, e2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn line_point_at() {
        let c = Curve::Line {
            origin: Point3::origin(),
            dir: Vector3::x(),
        };
        let p = c.point_at(2.5);
        assert!((p.x - 2.5).abs() < 1e-12);
        assert!(p.y.abs() < 1e-12);
        assert!(p.z.abs() < 1e-12);
    }

    #[test]
    fn circle_point_at_zero_is_on_circle() {
        let c = Curve::Circle {
            center: Point3::origin(),
            normal: Vector3::z(),
            radius: 3.0,
        };
        let p0 = c.point_at(0.0);
        let dist = (p0.coords.norm() - 3.0).abs();
        assert!(dist < 1e-12, "point should be on circle of radius 3, got dist={dist}");
    }

    #[test]
    fn sphere_surface_poles() {
        let s = Surface::Sphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        // North pole (v=0)
        let north = s.point_at(0.0, 0.0);
        assert!((north.z - 1.0).abs() < 1e-12);
        // South pole (v=π)
        let south = s.point_at(0.0, PI);
        assert!((south.z + 1.0).abs() < 1e-12);
    }

    #[test]
    fn plane_normal_at() {
        let s = Surface::Plane {
            origin: Point3::origin(),
            normal: Vector3::z(),
        };
        let n = s.normal_at(0.0, 0.0);
        assert!((n - Vector3::z()).norm() < 1e-12);
    }
}
