use eframe::egui_wgpu;
use rmsh_renderer::Scene;

/// The egui_wgpu callback that bridges egui and our standalone renderer.
pub struct ViewportCallback;

impl egui_wgpu::CallbackTrait for ViewportCallback {
    fn prepare(
        &self,
        _device: &wgpu::Device,
        queue: &wgpu::Queue,
        screen_descriptor: &egui_wgpu::ScreenDescriptor,
        _encoder: &mut wgpu::CommandEncoder,
        callback_resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        if let Some(scene) = callback_resources.get::<Scene>() {
            scene.update_uniforms(
                queue,
                screen_descriptor.size_in_pixels[0],
                screen_descriptor.size_in_pixels[1],
            );
        }
        Vec::new()
    }

    fn paint(
        &self,
        info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        callback_resources: &egui_wgpu::CallbackResources,
    ) {
        let Some(scene) = callback_resources.get::<Scene>() else {
            return;
        };

        let viewport = info.viewport_in_pixels();
        let width = viewport.width_px as u32;
        let height = viewport.height_px as u32;

        if width == 0 || height == 0 {
            return;
        }

        render_pass.set_viewport(
            viewport.left_px as f32,
            viewport.top_px as f32,
            width as f32,
            height as f32,
            0.0,
            1.0,
        );

        // Render mesh surface
        if scene.config.show_faces || scene.config.show_volumes {
            if let Some(ref surface) = scene.surface_gpu {
                render_pass.set_pipeline(scene.mesh_pipeline());
                render_pass.set_bind_group(0, scene.uniform_bind_group(), &[]);
                render_pass.set_vertex_buffer(0, surface.vertex_buffer.slice(..));
                render_pass.set_index_buffer(surface.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(0..surface.index_count, 0, 0..1);
            }
        }

        // Render wireframe
        if scene.config.show_edges {
            if let Some(ref wireframe) = scene.wireframe_gpu {
                render_pass.set_pipeline(scene.wireframe_pipeline());
                render_pass.set_bind_group(0, scene.uniform_bind_group(), &[]);
                render_pass.set_vertex_buffer(0, wireframe.vertex_buffer.slice(..));
                render_pass.set_index_buffer(wireframe.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(0..wireframe.index_count, 0, 0..1);
            }
        }

        // Render nodes
        if scene.config.show_nodes {
            if let Some(ref points) = scene.points_gpu {
                render_pass.set_pipeline(scene.point_pipeline());
                render_pass.set_bind_group(0, scene.uniform_bind_group(), &[]);
                render_pass.set_vertex_buffer(0, points.vertex_buffer.slice(..));
                render_pass.draw(0..6, 0..points.vertex_count);
            }
        }

        // Render highlight overlay (selected topology entity)
        if let Some(ref hl) = scene.highlight_gpu {
            if let Some(ref surface) = hl.surface {
                render_pass.set_pipeline(scene.highlight_surface_pipeline());
                render_pass.set_bind_group(0, scene.uniform_bind_group(), &[]);
                render_pass.set_vertex_buffer(0, surface.vertex_buffer.slice(..));
                render_pass.set_index_buffer(surface.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(0..surface.index_count, 0, 0..1);
            }
            if let Some(ref wireframe) = hl.wireframe {
                render_pass.set_pipeline(scene.highlight_wireframe_pipeline());
                render_pass.set_bind_group(0, scene.uniform_bind_group(), &[]);
                render_pass.set_vertex_buffer(0, wireframe.vertex_buffer.slice(..));
                render_pass.set_index_buffer(wireframe.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(0..wireframe.index_count, 0, 0..1);
            }
        }

        // Render gizmo in bottom-right corner
        if scene.config.show_gizmo {
            let gizmo_size = 96u32;
            let margin = 10u32;
            let gizmo_x = viewport.left_px as f32 + width as f32 - gizmo_size as f32 - margin as f32;
            let gizmo_y = viewport.top_px as f32 + height as f32 - gizmo_size as f32 - margin as f32;
            render_pass.set_viewport(gizmo_x, gizmo_y, gizmo_size as f32, gizmo_size as f32, 0.0, 1.0);
            render_pass.set_pipeline(scene.gizmo_pipeline());
            render_pass.set_bind_group(0, scene.gizmo_bind_group(), &[]);
            render_pass.set_vertex_buffer(0, scene.gizmo_vertex_buffer().slice(..));
            render_pass.draw(0..scene.gizmo_vertex_count(), 0..1);
        }
    }
}
