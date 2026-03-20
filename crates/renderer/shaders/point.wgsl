// Point shader — renders nodes as small billboarded quads

struct ViewUniforms {
    view_proj: mat4x4<f32>,
    model: mat4x4<f32>,
    camera_pos: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> uniforms: ViewUniforms;

struct VertexInput {
    @location(0) center: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
};

// Expand each point into a quad using 6 vertices (2 triangles).
// vertex_index % 6 determines which corner of the quad.
@vertex
fn vs_main(in: VertexInput, @builtin(vertex_index) vid: u32) -> VertexOutput {
    var out: VertexOutput;
    let world_pos = uniforms.model * vec4<f32>(in.center, 1.0);
    let clip_pos = uniforms.view_proj * world_pos;

    // Offsets for a quad in NDC (pixel size ~4px, adjusted by w for perspective)
    let point_size = 4.0;
    let offsets = array<vec2<f32>, 6>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>( 1.0, -1.0),
        vec2<f32>(-1.0,  1.0),
        vec2<f32>(-1.0,  1.0),
        vec2<f32>( 1.0, -1.0),
        vec2<f32>( 1.0,  1.0),
    );
    let corner = offsets[vid % 6u];
    // Scale offset to pixel size (assume ~1000px viewport)
    let scale = point_size / 1000.0;
    out.clip_position = vec4<f32>(
        clip_pos.x + corner.x * scale * clip_pos.w,
        clip_pos.y + corner.y * scale * clip_pos.w,
        clip_pos.z,
        clip_pos.w,
    );
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    return vec4<f32>(1.0, 0.3, 0.1, 1.0);
}
