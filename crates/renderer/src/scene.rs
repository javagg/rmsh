use wgpu::util::DeviceExt;

use crate::camera::OrbitCamera;
use crate::gizmo::AxisGizmo;
use crate::mesh_render::{HighlightGpu, MeshPointsGpu, MeshSurfaceGpu, MeshWireframeGpu};
use crate::pipeline;
use crate::uniform::ViewUniforms;

// Re-export for use from viewer crate
pub use crate::mesh_render;

/// Rendering configuration — controls what elements are visible.
#[derive(Debug, Clone)]
pub struct RenderConfig {
    pub show_nodes: bool,
    pub show_edges: bool,
    pub show_faces: bool,
    pub show_volumes: bool,
    pub show_gizmo: bool,
    /// Surface opacity (0.0 = transparent, 1.0 = opaque)
    pub surface_opacity: f32,
    /// Background color
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

/// The 3D scene — owns all GPU resources and render state.
/// This is independent of the UI framework.
pub struct Scene {
    pub camera: OrbitCamera,
    pub config: RenderConfig,

    // GPU resources
    uniform_buffer: wgpu::Buffer,
    uniform_bind_group: wgpu::BindGroup,

    mesh_pipeline: wgpu::RenderPipeline,
    wireframe_pipeline: wgpu::RenderPipeline,
    point_pipeline: wgpu::RenderPipeline,
    gizmo_pipeline: wgpu::RenderPipeline,
    highlight_surface_pipeline: wgpu::RenderPipeline,
    highlight_wireframe_pipeline: wgpu::RenderPipeline,

    // Gizmo
    gizmo: AxisGizmo,
    gizmo_uniform_buffer: wgpu::Buffer,
    gizmo_bind_group: wgpu::BindGroup,

    // Mesh GPU data
    pub surface_gpu: Option<MeshSurfaceGpu>,
    pub wireframe_gpu: Option<MeshWireframeGpu>,
    pub points_gpu: Option<MeshPointsGpu>,

    // Highlight GPU data (selected topology entity)
    pub highlight_gpu: Option<HighlightGpu>,
}

impl Scene {
    pub fn new(device: &wgpu::Device, target_format: wgpu::TextureFormat) -> Self {
        let bind_group_layout = pipeline::create_uniform_bind_group_layout(device);

        let uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("view_uniform_buffer"),
            contents: bytemuck::bytes_of(&ViewUniforms::default()),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("view_bind_group"),
            layout: &bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            }],
        });

        let gizmo_uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("gizmo_uniform_buffer"),
            contents: bytemuck::bytes_of(&ViewUniforms::default()),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let gizmo_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("gizmo_bind_group"),
            layout: &bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: gizmo_uniform_buffer.as_entire_binding(),
            }],
        });

        let mesh_pipeline =
            pipeline::create_mesh_pipeline(device, target_format, &bind_group_layout);
        let wireframe_pipeline =
            pipeline::create_wireframe_pipeline(device, target_format, &bind_group_layout);
        let point_pipeline =
            pipeline::create_point_pipeline(device, target_format, &bind_group_layout);
        let gizmo_pipeline =
            pipeline::create_gizmo_pipeline(device, target_format, &bind_group_layout);
        let highlight_surface_pipeline =
            pipeline::create_highlight_surface_pipeline(device, target_format, &bind_group_layout);
        let highlight_wireframe_pipeline = pipeline::create_highlight_wireframe_pipeline(
            device,
            target_format,
            &bind_group_layout,
        );

        let gizmo = AxisGizmo::new(device);

        Self {
            camera: OrbitCamera::new(),
            config: RenderConfig::default(),
            uniform_buffer,
            uniform_bind_group,
            mesh_pipeline,
            wireframe_pipeline,
            point_pipeline,
            gizmo_pipeline,
            highlight_surface_pipeline,
            highlight_wireframe_pipeline,
            gizmo,
            gizmo_uniform_buffer,
            gizmo_bind_group,
            surface_gpu: None,
            wireframe_gpu: None,
            points_gpu: None,
            highlight_gpu: None,
        }
    }

    /// Upload mesh data to GPU from extracted geometry.
    pub fn upload_mesh(
        &mut self,
        device: &wgpu::Device,
        surface: &rmsh_geo::extract::SurfaceData,
        wireframe: &rmsh_geo::extract::WireframeData,
        points: &rmsh_geo::extract::PointData,
    ) {
        self.surface_gpu = MeshSurfaceGpu::from_surface_data(device, surface);
        self.wireframe_gpu = MeshWireframeGpu::from_wireframe_data(device, wireframe);
        self.points_gpu = MeshPointsGpu::from_point_data(device, points);
    }

    /// Update uniforms on the GPU. Call this before rendering (e.g., in prepare()).
    pub fn update_uniforms(&self, queue: &wgpu::Queue, width: u32, height: u32) {
        if width == 0 || height == 0 {
            return;
        }
        let aspect = width as f32 / height as f32;

        // Main view uniforms
        let eye = self.camera.eye_position();
        let view_proj = self.camera.view_projection_matrix(aspect);
        let uniforms = ViewUniforms {
            view_proj: view_proj.into(),
            model: nalgebra::Matrix4::<f32>::identity().into(),
            camera_pos: [eye.x, eye.y, eye.z, 1.0],
        };
        queue.write_buffer(&self.uniform_buffer, 0, bytemuck::bytes_of(&uniforms));

        // Gizmo uniforms
        let gizmo_vp = AxisGizmo::gizmo_view_proj(&self.camera, width as f32, height as f32);
        let gizmo_uniforms = ViewUniforms {
            view_proj: gizmo_vp.into(),
            model: nalgebra::Matrix4::<f32>::identity().into(),
            camera_pos: [0.0, 0.0, 3.0, 1.0],
        };
        queue.write_buffer(
            &self.gizmo_uniform_buffer,
            0,
            bytemuck::bytes_of(&gizmo_uniforms),
        );
    }

    // --- Accessors for inline rendering from egui callback ---

    pub fn mesh_pipeline(&self) -> &wgpu::RenderPipeline {
        &self.mesh_pipeline
    }

    pub fn wireframe_pipeline(&self) -> &wgpu::RenderPipeline {
        &self.wireframe_pipeline
    }

    pub fn point_pipeline(&self) -> &wgpu::RenderPipeline {
        &self.point_pipeline
    }

    pub fn gizmo_pipeline(&self) -> &wgpu::RenderPipeline {
        &self.gizmo_pipeline
    }

    pub fn uniform_bind_group(&self) -> &wgpu::BindGroup {
        &self.uniform_bind_group
    }

    pub fn gizmo_bind_group(&self) -> &wgpu::BindGroup {
        &self.gizmo_bind_group
    }

    pub fn gizmo_vertex_buffer(&self) -> &wgpu::Buffer {
        self.gizmo.vertex_buffer()
    }

    pub fn gizmo_vertex_count(&self) -> u32 {
        self.gizmo.vertex_count()
    }

    pub fn highlight_surface_pipeline(&self) -> &wgpu::RenderPipeline {
        &self.highlight_surface_pipeline
    }

    pub fn highlight_wireframe_pipeline(&self) -> &wgpu::RenderPipeline {
        &self.highlight_wireframe_pipeline
    }

    /// Upload highlight geometry for a selected topology entity.
    pub fn upload_highlight(
        &mut self,
        device: &wgpu::Device,
        surface: Option<&rmsh_geo::extract::SurfaceData>,
        wireframe: Option<&rmsh_geo::extract::WireframeData>,
    ) {
        let hl_surface = surface.and_then(|s| MeshSurfaceGpu::from_surface_data(device, s));
        let hl_wireframe = wireframe.and_then(|w| MeshWireframeGpu::from_wireframe_data(device, w));

        if hl_surface.is_some() || hl_wireframe.is_some() {
            self.highlight_gpu = Some(HighlightGpu {
                surface: hl_surface,
                wireframe: hl_wireframe,
            });
        } else {
            self.highlight_gpu = None;
        }
    }

    /// Clear highlight.
    pub fn clear_highlight(&mut self) {
        self.highlight_gpu = None;
    }
}
