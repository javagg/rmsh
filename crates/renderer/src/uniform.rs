use bytemuck::{Pod, Zeroable};

/// Uniform data sent to the GPU for view/projection transforms.
#[repr(C)]
#[derive(Debug, Copy, Clone, Pod, Zeroable)]
pub struct ViewUniforms {
    pub view_proj: [[f32; 4]; 4],
    pub model: [[f32; 4]; 4],
    pub camera_pos: [f32; 4],
}

impl Default for ViewUniforms {
    fn default() -> Self {
        Self {
            view_proj: nalgebra::Matrix4::<f32>::identity().into(),
            model: nalgebra::Matrix4::<f32>::identity().into(),
            camera_pos: [0.0, 0.0, 5.0, 1.0],
        }
    }
}
