use wgpu::util::DeviceExt;

use rmsh_geo::extract::{PointData, SurfaceData, WireframeData};

/// GPU buffers for mesh surface rendering.
pub struct MeshSurfaceGpu {
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
    pub index_count: u32,
}

/// GPU buffers for wireframe rendering.
pub struct MeshWireframeGpu {
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
    pub index_count: u32,
}

/// GPU buffers for point rendering.
pub struct MeshPointsGpu {
    pub vertex_buffer: wgpu::Buffer,
    pub vertex_count: u32,
}

impl MeshSurfaceGpu {
    pub fn from_surface_data(device: &wgpu::Device, data: &SurfaceData) -> Option<Self> {
        if data.indices.is_empty() {
            return None;
        }

        // Interleave position + normal into [f32; 6] per vertex
        let mut vertices: Vec<[f32; 6]> = Vec::with_capacity(data.positions.len());
        for i in 0..data.positions.len() {
            let p = data.positions[i];
            let n = data.normals[i];
            vertices.push([p[0], p[1], p[2], n[0], n[1], n[2]]);
        }

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh_surface_vertices"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh_surface_indices"),
            contents: bytemuck::cast_slice(&data.indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        Some(Self {
            vertex_buffer,
            index_buffer,
            index_count: data.indices.len() as u32,
        })
    }
}

impl MeshWireframeGpu {
    pub fn from_wireframe_data(device: &wgpu::Device, data: &WireframeData) -> Option<Self> {
        if data.indices.is_empty() {
            return None;
        }

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh_wireframe_vertices"),
            contents: bytemuck::cast_slice(&data.positions),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh_wireframe_indices"),
            contents: bytemuck::cast_slice(&data.indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        Some(Self {
            vertex_buffer,
            index_buffer,
            index_count: data.indices.len() as u32,
        })
    }
}

impl MeshPointsGpu {
    pub fn from_point_data(device: &wgpu::Device, data: &PointData) -> Option<Self> {
        if data.positions.is_empty() {
            return None;
        }

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh_points_vertices"),
            contents: bytemuck::cast_slice(&data.positions),
            usage: wgpu::BufferUsages::VERTEX,
        });

        Some(Self {
            vertex_buffer,
            vertex_count: data.positions.len() as u32,
        })
    }
}
