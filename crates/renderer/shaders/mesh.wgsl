// Mesh surface shader with Blinn-Phong lighting

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

    // Use face normal direction (flip if back-facing)
    var normal = normalize(in.world_normal);
    if (dot(normal, view_dir) < 0.0) {
        normal = -normal;
    }

    // Ambient
    let ambient_color = vec3<f32>(0.1, 0.1, 0.15);

    // Diffuse
    let diff = max(dot(normal, light_dir), 0.0);
    let diffuse_color = vec3<f32>(0.4, 0.6, 0.8) * diff;

    // Specular (Blinn-Phong)
    let half_dir = normalize(light_dir + view_dir);
    let spec = pow(max(dot(normal, half_dir), 0.0), 32.0);
    let specular_color = vec3<f32>(1.0, 1.0, 1.0) * spec * 0.3;

    let color = ambient_color + diffuse_color + specular_color;
    return vec4<f32>(color, 0.9);
}
