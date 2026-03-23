use wgpu::*;

/// Create the mesh surface render pipeline (triangles with lighting).
pub fn create_mesh_pipeline(
    device: &Device,
    format: TextureFormat,
    bind_group_layout: &BindGroupLayout,
) -> RenderPipeline {
    let shader = device.create_shader_module(ShaderModuleDescriptor {
        label: Some("mesh_shader"),
        source: ShaderSource::Wgsl(include_str!("../shaders/mesh.wgsl").into()),
    });

    let pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
        label: Some("mesh_pipeline_layout"),
        bind_group_layouts: &[bind_group_layout],
        push_constant_ranges: &[],
    });

    device.create_render_pipeline(&RenderPipelineDescriptor {
        label: Some("mesh_pipeline"),
        layout: Some(&pipeline_layout),
        vertex: VertexState {
            module: &shader,
            entry_point: Some("vs_main"),
            buffers: &[VertexBufferLayout {
                array_stride: std::mem::size_of::<[f32; 9]>() as BufferAddress,
                step_mode: VertexStepMode::Vertex,
                attributes: &vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x3],
            }],
            compilation_options: Default::default(),
        },
        fragment: Some(FragmentState {
            module: &shader,
            entry_point: Some("fs_main"),
            targets: &[Some(ColorTargetState {
                format,
                blend: Some(BlendState::ALPHA_BLENDING),
                write_mask: ColorWrites::ALL,
            })],
            compilation_options: Default::default(),
        }),
        primitive: PrimitiveState {
            topology: PrimitiveTopology::TriangleList,
            cull_mode: None, // Show both sides
            ..Default::default()
        },
        depth_stencil: None,
        multisample: MultisampleState::default(),
        multiview: None,
        cache: None,
    })
}

/// Create the wireframe render pipeline (lines).
pub fn create_wireframe_pipeline(
    device: &Device,
    format: TextureFormat,
    bind_group_layout: &BindGroupLayout,
) -> RenderPipeline {
    let shader = device.create_shader_module(ShaderModuleDescriptor {
        label: Some("wireframe_shader"),
        source: ShaderSource::Wgsl(include_str!("../shaders/wireframe.wgsl").into()),
    });

    let pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
        label: Some("wireframe_pipeline_layout"),
        bind_group_layouts: &[bind_group_layout],
        push_constant_ranges: &[],
    });

    device.create_render_pipeline(&RenderPipelineDescriptor {
        label: Some("wireframe_pipeline"),
        layout: Some(&pipeline_layout),
        vertex: VertexState {
            module: &shader,
            entry_point: Some("vs_main"),
            buffers: &[VertexBufferLayout {
                array_stride: std::mem::size_of::<[f32; 3]>() as BufferAddress,
                step_mode: VertexStepMode::Vertex,
                attributes: &vertex_attr_array![0 => Float32x3],
            }],
            compilation_options: Default::default(),
        },
        fragment: Some(FragmentState {
            module: &shader,
            entry_point: Some("fs_main"),
            targets: &[Some(ColorTargetState {
                format,
                blend: Some(BlendState::REPLACE),
                write_mask: ColorWrites::ALL,
            })],
            compilation_options: Default::default(),
        }),
        primitive: PrimitiveState {
            topology: PrimitiveTopology::LineList,
            ..Default::default()
        },
        depth_stencil: None,
        multisample: MultisampleState::default(),
        multiview: None,
        cache: None,
    })
}

/// Create the point render pipeline for rendering nodes.
pub fn create_point_pipeline(
    device: &Device,
    format: TextureFormat,
    bind_group_layout: &BindGroupLayout,
) -> RenderPipeline {
    let shader = device.create_shader_module(ShaderModuleDescriptor {
        label: Some("point_shader"),
        source: ShaderSource::Wgsl(include_str!("../shaders/point.wgsl").into()),
    });

    let pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
        label: Some("point_pipeline_layout"),
        bind_group_layouts: &[bind_group_layout],
        push_constant_ranges: &[],
    });

    device.create_render_pipeline(&RenderPipelineDescriptor {
        label: Some("point_pipeline"),
        layout: Some(&pipeline_layout),
        vertex: VertexState {
            module: &shader,
            entry_point: Some("vs_main"),
            buffers: &[VertexBufferLayout {
                array_stride: std::mem::size_of::<[f32; 3]>() as BufferAddress,
                step_mode: VertexStepMode::Instance,
                attributes: &vertex_attr_array![0 => Float32x3],
            }],
            compilation_options: Default::default(),
        },
        fragment: Some(FragmentState {
            module: &shader,
            entry_point: Some("fs_main"),
            targets: &[Some(ColorTargetState {
                format,
                blend: Some(BlendState::REPLACE),
                write_mask: ColorWrites::ALL,
            })],
            compilation_options: Default::default(),
        }),
        primitive: PrimitiveState {
            topology: PrimitiveTopology::TriangleList,
            ..Default::default()
        },
        depth_stencil: None,
        multisample: MultisampleState::default(),
        multiview: None,
        cache: None,
    })
}

/// Create the axis gizmo pipeline.
pub fn create_gizmo_pipeline(
    device: &Device,
    format: TextureFormat,
    bind_group_layout: &BindGroupLayout,
) -> RenderPipeline {
    let shader = device.create_shader_module(ShaderModuleDescriptor {
        label: Some("gizmo_shader"),
        source: ShaderSource::Wgsl(include_str!("../shaders/gizmo.wgsl").into()),
    });

    let pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
        label: Some("gizmo_pipeline_layout"),
        bind_group_layouts: &[bind_group_layout],
        push_constant_ranges: &[],
    });

    device.create_render_pipeline(&RenderPipelineDescriptor {
        label: Some("gizmo_pipeline"),
        layout: Some(&pipeline_layout),
        vertex: VertexState {
            module: &shader,
            entry_point: Some("vs_main"),
            buffers: &[VertexBufferLayout {
                array_stride: std::mem::size_of::<[f32; 6]>() as BufferAddress,
                step_mode: VertexStepMode::Vertex,
                attributes: &vertex_attr_array![0 => Float32x3, 1 => Float32x3],
            }],
            compilation_options: Default::default(),
        },
        fragment: Some(FragmentState {
            module: &shader,
            entry_point: Some("fs_main"),
            targets: &[Some(ColorTargetState {
                format,
                blend: Some(BlendState::REPLACE),
                write_mask: ColorWrites::ALL,
            })],
            compilation_options: Default::default(),
        }),
        primitive: PrimitiveState {
            topology: PrimitiveTopology::LineList,
            ..Default::default()
        },
        depth_stencil: None, // Gizmo always on top
        multisample: MultisampleState::default(),
        multiview: None,
        cache: None,
    })
}

/// Create the uniform bind group layout.
pub fn create_uniform_bind_group_layout(device: &Device) -> BindGroupLayout {
    device.create_bind_group_layout(&BindGroupLayoutDescriptor {
        label: Some("uniform_bind_group_layout"),
        entries: &[BindGroupLayoutEntry {
            binding: 0,
            visibility: ShaderStages::VERTEX | ShaderStages::FRAGMENT,
            ty: BindingType::Buffer {
                ty: BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        }],
    })
}

/// Create the highlight surface pipeline (selected faces drawn on top in highlight color).
pub fn create_highlight_surface_pipeline(
    device: &Device,
    format: TextureFormat,
    bind_group_layout: &BindGroupLayout,
) -> RenderPipeline {
    let shader = device.create_shader_module(ShaderModuleDescriptor {
        label: Some("highlight_shader"),
        source: ShaderSource::Wgsl(include_str!("../shaders/highlight.wgsl").into()),
    });

    let pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
        label: Some("highlight_pipeline_layout"),
        bind_group_layouts: &[bind_group_layout],
        push_constant_ranges: &[],
    });

    device.create_render_pipeline(&RenderPipelineDescriptor {
        label: Some("highlight_surface_pipeline"),
        layout: Some(&pipeline_layout),
        vertex: VertexState {
            module: &shader,
            entry_point: Some("vs_main"),
            buffers: &[VertexBufferLayout {
                array_stride: std::mem::size_of::<[f32; 9]>() as BufferAddress,
                step_mode: VertexStepMode::Vertex,
                attributes: &vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x3],
            }],
            compilation_options: Default::default(),
        },
        fragment: Some(FragmentState {
            module: &shader,
            entry_point: Some("fs_main"),
            targets: &[Some(ColorTargetState {
                format,
                blend: Some(BlendState::ALPHA_BLENDING),
                write_mask: ColorWrites::ALL,
            })],
            compilation_options: Default::default(),
        }),
        primitive: PrimitiveState {
            topology: PrimitiveTopology::TriangleList,
            cull_mode: None,
            ..Default::default()
        },
        depth_stencil: None,
        multisample: MultisampleState::default(),
        multiview: None,
        cache: None,
    })
}

/// Create the highlight wireframe pipeline (selected edges drawn in highlight color).
pub fn create_highlight_wireframe_pipeline(
    device: &Device,
    format: TextureFormat,
    bind_group_layout: &BindGroupLayout,
) -> RenderPipeline {
    let shader = device.create_shader_module(ShaderModuleDescriptor {
        label: Some("highlight_wire_shader"),
        source: ShaderSource::Wgsl(include_str!("../shaders/wireframe.wgsl").into()),
    });

    let pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
        label: Some("highlight_wire_pipeline_layout"),
        bind_group_layouts: &[bind_group_layout],
        push_constant_ranges: &[],
    });

    device.create_render_pipeline(&RenderPipelineDescriptor {
        label: Some("highlight_wireframe_pipeline"),
        layout: Some(&pipeline_layout),
        vertex: VertexState {
            module: &shader,
            entry_point: Some("vs_main"),
            buffers: &[VertexBufferLayout {
                array_stride: std::mem::size_of::<[f32; 3]>() as BufferAddress,
                step_mode: VertexStepMode::Vertex,
                attributes: &vertex_attr_array![0 => Float32x3],
            }],
            compilation_options: Default::default(),
        },
        fragment: Some(FragmentState {
            module: &shader,
            entry_point: Some("fs_main"),
            targets: &[Some(ColorTargetState {
                format,
                blend: Some(BlendState::REPLACE),
                write_mask: ColorWrites::ALL,
            })],
            compilation_options: Default::default(),
        }),
        primitive: PrimitiveState {
            topology: PrimitiveTopology::LineList,
            ..Default::default()
        },
        depth_stencil: None,
        multisample: MultisampleState::default(),
        multiview: None,
        cache: None,
    })
}
