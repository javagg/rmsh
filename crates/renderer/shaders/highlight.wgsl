// Highlight shader — renders selected faces/edges/points with a solid highlight color.
// Reuses the same ViewUniforms as the mesh shader.

struct ViewUniforms {
    view_proj: mat4x4<f32>,
    model: mat4x4<f32>,
    camera_pos: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> uniforms: ViewUniforms;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_normal: vec3<f32>,
    @location(1) world_position: vec3<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    let world_pos = uniforms.model * vec4<f32>(in.position, 1.0);
    out.clip_position = uniforms.view_proj * world_pos;
    out.world_normal = normalize((uniforms.model * vec4<f32>(in.normal, 0.0)).xyz);
    out.world_position = world_pos.xyz;
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let light_dir = normalize(vec3<f32>(0.5, 1.0, 0.8));
    let view_dir = normalize(uniforms.camera_pos.xyz - in.world_position);

    var normal = normalize(in.world_normal);
    if (dot(normal, view_dir) < 0.0) {
        normal = -normal;
    }

    // Simple shaded highlight in orange-ish color
    let ambient = vec3<f32>(0.15, 0.08, 0.02);
    let diff = max(dot(normal, light_dir), 0.0);
    let diffuse = vec3<f32>(1.0, 0.6, 0.1) * diff;

    let half_dir = normalize(light_dir + view_dir);
    let spec = pow(max(dot(normal, half_dir), 0.0), 32.0);
    let specular = vec3<f32>(1.0, 0.9, 0.7) * spec * 0.4;

    let color = ambient + diffuse + specular;
    return vec4<f32>(color, 0.95);
}
