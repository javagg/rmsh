use rcad_render::{Camera, Tessellator, WgpuRenderer};
use rcad_kernel::{BRep, Shell, Solid, Vertex, Wire, Face};
use rmsh_geo::extract::{PointData, SurfaceData, WireframeData};

use crate::gizmo::GizmoRenderer;

/// Re-export rcad-render Camera as the orbit camera.
pub use rcad_render::Camera as OrbitCamera;

/// Extension methods for `Camera` (removed from rcad-render upstream).
pub trait CameraExt {
    fn rotate(&mut self, delta_yaw: f32, delta_pitch: f32);
    fn zoom(&mut self, delta: f32);
    fn pan(&mut self, delta_x: f32, delta_y: f32);
    fn fit_to_bbox(&mut self, center: [f32; 3], diagonal: f32);
    fn set_isometric(&mut self);
    fn toggle_projection(&mut self);
    fn orthographic(&self) -> bool;
}

impl CameraExt for Camera {
    fn rotate(&mut self, delta_yaw: f32, delta_pitch: f32) {
        self.rot_y += delta_yaw;
        self.rot_x = (self.rot_x + delta_pitch).clamp(
            -std::f32::consts::FRAC_PI_2 + 0.01,
            std::f32::consts::FRAC_PI_2 - 0.01,
        );
    }

    fn zoom(&mut self, delta: f32) {
        self.distance = (self.distance * (1.0 - delta)).max(0.01);
    }

    fn pan(&mut self, delta_x: f32, delta_y: f32) {
        self.pan_pixels(delta_x, delta_y);
    }

    fn fit_to_bbox(&mut self, center: [f32; 3], diagonal: f32) {
        self.target = glam::Vec3::new(center[0], center[1], center[2]);
        self.distance = diagonal * 1.5;
    }

    fn set_isometric(&mut self) {
        self.rot_x = 0.6154_8246; // atan(1/sqrt(2))
        self.rot_y = std::f32::consts::FRAC_PI_4;
    }

    fn toggle_projection(&mut self) {
        // Perspective-only camera; no-op.
    }

    fn orthographic(&self) -> bool {
        false
    }
}

/// Rendering configuration — controls what elements are visible.
#[derive(Debug, Clone)]
pub struct RenderConfig {
    pub show_nodes: bool,
    pub show_edges: bool,
    pub show_faces: bool,
    pub show_volumes: bool,
    pub show_gizmo: bool,
    pub surface_opacity: f32,
    pub bg_color: [f32; 4],
}

impl Default for RenderConfig {
    fn default() -> Self {
        Self {
            show_nodes: false,
            show_edges: true,
            show_faces: true,
            show_volumes: true,
            show_gizmo: true,
            surface_opacity: 0.9,
            bg_color: [0.15, 0.15, 0.18, 1.0],
        }
    }
}

/// The 3D scene — wraps `rcad_render::WgpuRenderer` and bridges rmsh geometry types.
pub struct Scene {
    pub camera: Camera,
    pub config: RenderConfig,
    pub renderer: WgpuRenderer,
    pub gizmo: GizmoRenderer,
}

impl Scene {
    pub fn new(device: &wgpu::Device, target_format: wgpu::TextureFormat) -> Self {
        Self {
            camera: Camera::new(),
            config: RenderConfig::default(),
            renderer: WgpuRenderer::new(device, target_format),
            gizmo: GizmoRenderer::new(device, target_format),
        }
    }

    /// Upload mesh data to GPU from extracted geometry.
    pub fn upload_mesh(
        &mut self,
        device: &wgpu::Device,
        surface: &SurfaceData,
        wireframe: &WireframeData,
        _points: &PointData,
    ) {
        let brep = surface_wireframe_to_brep(surface, wireframe);
        let mesh = Tessellator::tessellate(&brep);
        self.renderer.upload_mesh(device, &mesh);
    }

    /// Upload highlight geometry.
    pub fn upload_highlight(
        &mut self,
        device: &wgpu::Device,
        surface: Option<&SurfaceData>,
        wireframe: Option<&WireframeData>,
    ) {
        let face_mesh = surface.map(|s| {
            let brep = surface_to_brep(s);
            Tessellator::tessellate(&brep)
        });
        let edge_mesh = wireframe.map(|w| {
            let brep = wireframe_to_brep(w);
            Tessellator::tessellate(&brep)
        });
        self.renderer.upload_highlights(device, face_mesh.as_ref(), edge_mesh.as_ref());
    }

    /// Clear highlight geometry.
    pub fn clear_highlight(&mut self, device: &wgpu::Device) {
        self.renderer.upload_highlights(device, None, None);
    }

    /// Update camera uniforms (called from egui prepare callback).
    pub fn update_uniforms(&self, queue: &wgpu::Queue, width: u32, height: u32) {
        if width == 0 || height == 0 {
            return;
        }
        let aspect = width as f32 / height as f32;
        self.renderer.update_camera(queue, &self.camera, aspect);
        if self.config.show_gizmo {
            self.gizmo.update(queue, &self.camera, width, height);
        }
    }

    /// Draw into an active render pass (called from egui paint callback).
    pub fn draw_in_render_pass(&self, render_pass: &mut wgpu::RenderPass<'_>) {
        self.renderer.draw_in_render_pass(render_pass, false);
        if self.config.show_gizmo {
            self.gizmo.draw(render_pass);
        }
    }
}

// ── Geometry conversion helpers ───────────────────────────────────────────────

/// Convert SurfaceData + WireframeData into a BRep for rcad-render tessellation.
fn surface_wireframe_to_brep(surface: &SurfaceData, wireframe: &WireframeData) -> BRep {
    let vertices: Vec<Vertex> = surface
        .positions
        .iter()
        .map(|p| Vertex {
            point: glam::DVec3::new(p[0] as f64, p[1] as f64, p[2] as f64),
        })
        .collect();

    let triangles: Vec<[usize; 3]> = surface.indices
        .chunks_exact(3)
        .map(|c| [c[0] as usize, c[1] as usize, c[2] as usize])
        .collect();

    let edges: Vec<rcad_kernel::Edge> = wireframe.indices
        .chunks_exact(2)
        .map(|c| rcad_kernel::Edge { start: c[0] as usize, end: c[1] as usize })
        .collect();

    let face = Face {
        outer_wire: Wire { edges: Vec::new() },
        inner_wires: Vec::new(),
        normal: glam::DVec3::Z,
        triangles,
    };

    BRep {
        vertices,
        edges,
        solids: vec![Solid {
            shells: vec![Shell { faces: vec![face] }],
        }],
        geom: rcad_kernel::GeomStore::default(),
    }
}

fn surface_to_brep(surface: &SurfaceData) -> BRep {
    let empty_wireframe = WireframeData { positions: Vec::new(), indices: Vec::new() };
    surface_wireframe_to_brep(surface, &empty_wireframe)
}

fn wireframe_to_brep(wireframe: &WireframeData) -> BRep {
    let vertices: Vec<Vertex> = wireframe
        .positions
        .iter()
        .map(|p| Vertex {
            point: glam::DVec3::new(p[0] as f64, p[1] as f64, p[2] as f64),
        })
        .collect();

    let edges: Vec<rcad_kernel::Edge> = wireframe.indices
        .chunks_exact(2)
        .map(|c| rcad_kernel::Edge { start: c[0] as usize, end: c[1] as usize })
        .collect();

    BRep {
        vertices,
        edges,
        solids: Vec::new(),
        geom: rcad_kernel::GeomStore::default(),
    }
}
