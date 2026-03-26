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
        let letter_gap = 0.08f32;
        let letter_size = 0.16f32;

        fn push_segment(
            vertices: &mut Vec<GizmoVertex>,
            a: [f32; 3],
            b: [f32; 3],
            color: [f32; 3],
        ) {
            vertices.push(GizmoVertex { position: a, color });
            vertices.push(GizmoVertex { position: b, color });
        }

        // X axis (red), Y axis (green), Z axis (bright blue)
        let x_color = [1.0, 0.15, 0.15];
        let y_color = [0.2, 1.0, 0.2];
        let z_color = [0.2, 0.65, 1.0];

        let mut vertices = Vec::new();

        // Axes
        push_segment(
            &mut vertices,
            [0.0, 0.0, 0.0],
            [axis_length, 0.0, 0.0],
            x_color,
        );
        push_segment(
            &mut vertices,
            [0.0, 0.0, 0.0],
            [0.0, axis_length, 0.0],
            y_color,
        );
        push_segment(
            &mut vertices,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, axis_length],
            z_color,
        );

        // X glyph near +X endpoint (drawn in YZ plane)
        let x0 = axis_length + letter_gap;
        let hs = letter_size * 0.5;
        push_segment(&mut vertices, [x0, -hs, -hs], [x0, hs, hs], x_color);
        push_segment(&mut vertices, [x0, -hs, hs], [x0, hs, -hs], x_color);

        // Y glyph near +Y endpoint (drawn in XY plane)
        let y0 = axis_length + letter_gap;
        push_segment(
            &mut vertices,
            [-hs, y0 + hs * 0.6, 0.0],
            [0.0, y0, 0.0],
            y_color,
        );
        push_segment(
            &mut vertices,
            [hs, y0 + hs * 0.6, 0.0],
            [0.0, y0, 0.0],
            y_color,
        );
        push_segment(&mut vertices, [0.0, y0, 0.0], [0.0, y0 - hs, 0.0], y_color);

        // Z glyph near +Z endpoint (drawn in XY plane)
        let z0 = axis_length + letter_gap;
        push_segment(&mut vertices, [-hs, hs, z0], [hs, hs, z0], z_color);
        push_segment(&mut vertices, [hs, hs, z0], [-hs, -hs, z0], z_color);
        push_segment(&mut vertices, [-hs, -hs, z0], [hs, -hs, z0], z_color);

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
