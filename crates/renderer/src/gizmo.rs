use bytemuck;
use wgpu::util::DeviceExt;

/// Axis gizmo — renders XYZ coordinate axes in a corner of the viewport.
pub struct AxisGizmo {
    vertex_buffer: wgpu::Buffer,
    vertex_count: u32,
}

/// Gizmo vertex: position + color
#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct GizmoVertex {
    position: [f32; 3],
    color: [f32; 3],
}

impl AxisGizmo {
    pub fn new(device: &wgpu::Device) -> Self {
        let axis_length = 1.0f32;
        // X axis (red), Y axis (green), Z axis (blue)
        let vertices = vec![
            // X axis
            GizmoVertex { position: [0.0, 0.0, 0.0], color: [1.0, 0.0, 0.0] },
            GizmoVertex { position: [axis_length, 0.0, 0.0], color: [1.0, 0.0, 0.0] },
            // Y axis
            GizmoVertex { position: [0.0, 0.0, 0.0], color: [0.0, 1.0, 0.0] },
            GizmoVertex { position: [0.0, axis_length, 0.0], color: [0.0, 1.0, 0.0] },
            // Z axis
            GizmoVertex { position: [0.0, 0.0, 0.0], color: [0.3, 0.3, 1.0] },
            GizmoVertex { position: [0.0, 0.0, axis_length], color: [0.3, 0.3, 1.0] },
        ];

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("gizmo_vertex_buffer"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        Self {
            vertex_buffer,
            vertex_count: vertices.len() as u32,
        }
    }

    /// Compute the view-projection matrix for the gizmo.
    /// The gizmo tracks camera rotation but is placed in a fixed corner.
    pub fn gizmo_view_proj(
        camera: &crate::camera::OrbitCamera,
        viewport_width: f32,
        viewport_height: f32,
    ) -> nalgebra::Matrix4<f32> {
        // Gizmo occupies bottom-left corner — use camera rotation only
        let eye = nalgebra::Point3::new(
            camera.pitch.cos() * camera.yaw.sin() * 3.0,
            camera.pitch.sin() * 3.0,
            camera.pitch.cos() * camera.yaw.cos() * 3.0,
        );
        let view = nalgebra::Matrix4::look_at_rh(
            &eye,
            &nalgebra::Point3::origin(),
            &nalgebra::Vector3::y(),
        );
        let aspect = viewport_width / viewport_height;
        let proj = nalgebra::Matrix4::new_orthographic(-aspect, aspect, -1.0, 1.0, 0.1, 100.0);
        proj * view
    }

    pub fn vertex_buffer(&self) -> &wgpu::Buffer {
        &self.vertex_buffer
    }

    pub fn vertex_count(&self) -> u32 {
        self.vertex_count
    }
}
