use rcad_render::{Camera, GizmoRenderer, Tessellator, WgpuRenderer};
use rcad_kernel::{BRep, Shell, Solid, Vertex, Wire, Face};
use rmsh_geo::extract::{PointData, SurfaceData, WireframeData};

/// Re-export rcad-render Camera as the orbit camera.
pub use rcad_render::Camera as OrbitCamera;

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
    _highlight_clear_pending: bool,
}

impl Scene {
    pub fn new(device: &wgpu::Device, target_format: wgpu::TextureFormat) -> Self {
        Self {
            camera: Camera::new(),
            config: RenderConfig::default(),
            renderer: WgpuRenderer::new(device, target_format),
            gizmo: GizmoRenderer::new(device, target_format),
            _highlight_clear_pending: false,
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
        self._highlight_clear_pending = false;
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

    /// Clear highlight.
    pub fn clear_highlight(&mut self) {
        // Mark as dirty; actual GPU clear happens on next upload_highlight call.
        // rcad-render clears highlights when None meshes are passed — but we need
        // a device reference for that. Store a pending-clear flag instead.
        self._highlight_clear_pending = true;
    }

    /// Clear highlight with device (performs immediate GPU buffer clear).
    pub fn clear_highlight_with_device(&mut self, device: &wgpu::Device) {
        self.renderer.upload_highlights(device, None, None);
        self._highlight_clear_pending = false;
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

    // Build triangle faces from the surface index buffer
    let mut triangles: Vec<[usize; 3]> = Vec::new();
    let idx = &surface.indices;
    let mut i = 0;
    while i + 2 < idx.len() {
        triangles.push([idx[i] as usize, idx[i + 1] as usize, idx[i + 2] as usize]);
        i += 3;
    }

    // Build edge list (line_indices are pairs)
    let mut edges: Vec<rcad_kernel::Edge> = Vec::new();
    let widx = &wireframe.indices;
    let mut wi = 0;
    while wi + 1 < widx.len() {
        edges.push(rcad_kernel::Edge {
            start: widx[wi] as usize,
            end: widx[wi + 1] as usize,
        });
        wi += 2;
    }

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
    // Build minimal vertex list from wireframe positions
    let vertices: Vec<Vertex> = wireframe
        .positions
        .iter()
        .map(|p| Vertex {
            point: glam::DVec3::new(p[0] as f64, p[1] as f64, p[2] as f64),
        })
        .collect();

    let mut edges: Vec<rcad_kernel::Edge> = Vec::new();
    let widx = &wireframe.indices;
    let mut wi = 0;
    while wi + 1 < widx.len() {
        edges.push(rcad_kernel::Edge {
            start: widx[wi] as usize,
            end: widx[wi + 1] as usize,
        });
        wi += 2;
    }

    BRep {
        vertices,
        edges,
        solids: Vec::new(),
        geom: rcad_kernel::GeomStore::default(),
    }
}
