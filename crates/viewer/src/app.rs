use eframe::egui_wgpu;
use rmsh_model::Mesh;
use rmsh_renderer::{RenderConfig, Scene};

use crate::io::{
    default_save_name, drain_io_events, new_io_queue, request_open_dialog, request_open_path,
    request_save_dialog, IoEvent, IoQueue, MshSaveFormat,
};
use crate::viewport::ViewportCallback;

/// The main application state.
pub struct RmshApp {
    /// Currently loaded mesh.
    mesh: Option<Mesh>,
    /// Last opened mesh file name, if any.
    mesh_name: Option<String>,
    /// Render configuration (what to show).
    config: RenderConfig,
    /// Mesh info string for status bar.
    mesh_info: String,
    /// Whether the scene has been initialized with GPU resources.
    scene_initialized: bool,
    /// Cached wgpu render state.
    render_state: Option<egui_wgpu::RenderState>,
    /// Pending IO events from dialogs and drag-and-drop.
    io_queue: IoQueue,
}

impl RmshApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // Initialize the Scene in the wgpu render state callback resources
        if let Some(render_state) = cc.wgpu_render_state.as_ref() {
            let device = &render_state.device;
            let format = render_state.target_format;
            let scene = Scene::new(device, format);
            render_state
                .renderer
                .write()
                .callback_resources
                .insert(scene);
        }

        let render_state = cc.wgpu_render_state.clone();

        Self {
            mesh: None,
            mesh_name: None,
            config: RenderConfig::default(),
            mesh_info: String::new(),
            scene_initialized: false,
            render_state,
            io_queue: new_io_queue(),
        }
    }

    fn apply_loaded_mesh(&mut self, file_name: &str, data: &[u8]) -> anyhow::Result<()> {
        let mesh = rmsh_io::load_msh_from_bytes(data)?;
        self.mesh_info = format!(
            "Nodes: {}  Elements: {}  File: {}",
            mesh.node_count(),
            mesh.element_count(),
            file_name
        );
        self.mesh = Some(mesh);
        self.mesh_name = Some(file_name.to_string());
        self.scene_initialized = false;
        Ok(())
    }

    fn upload_mesh_to_gpu(&mut self, render_state: &egui_wgpu::RenderState) {
        if self.scene_initialized {
            return;
        }
        let Some(mesh) = &self.mesh else { return };

        let device = &render_state.device;

        // Extract geometry
        let surface = rmsh_geo::extract::extract_surface(mesh);
        let wireframe = rmsh_geo::extract::extract_wireframe(mesh, &[1, 2, 3]);
        let points = rmsh_geo::extract::extract_points(mesh);

        // Upload to GPU and fit camera
        let mut renderer = render_state.renderer.write();
        if let Some(scene) = renderer.callback_resources.get_mut::<Scene>() {
            scene.upload_mesh(device, &surface, &wireframe, &points);

            // Fit camera to mesh
            let center = mesh.center();
            let diag = mesh.diagonal_length() as f32;
            scene.camera.fit_to_bbox(
                [center.x as f32, center.y as f32, center.z as f32],
                diag,
            );
        }
        self.scene_initialized = true;
    }
}

impl eframe::App for RmshApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        for event in drain_io_events(&self.io_queue) {
            match event {
                IoEvent::MeshLoaded { file_name, data } => {
                    match self.apply_loaded_mesh(&file_name, &data) {
                        Ok(()) => log::info!("Loaded mesh: {}", file_name),
                        Err(e) => log::error!("Failed to load mesh: {}", e),
                    }
                }
                IoEvent::Error(message) => {
                    log::error!("{}", message);
                }
            }
        }

        // Upload mesh to GPU if needed
        if let Some(render_state) = self.render_state.clone() {
            self.upload_mesh_to_gpu(&render_state);

            // Sync config to scene
            let mut renderer = render_state.renderer.write();
            if let Some(scene) = renderer.callback_resources.get_mut::<Scene>() {
                scene.config = self.config.clone();
            }
        }

        // Handle file drop
        ctx.input(|i| {
            if !i.raw.dropped_files.is_empty() {
                if let Some(path) = i.raw.dropped_files[0].path.as_ref() {
                    request_open_path(path.clone(), self.io_queue.clone(), ctx.clone());
                }
            }
        });

        // Menu bar
        egui::TopBottomPanel::top("menu_bar").show(ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                ui.menu_button("File", |ui| {
                    if ui.button("Open MSH...").clicked() {
                        request_open_dialog(self.io_queue.clone(), ctx.clone());
                        ui.close_menu();
                    }
                    if ui
                        .add_enabled(self.mesh.is_some(), egui::Button::new("Save As MSH 4.1..."))
                        .clicked()
                    {
                        if let Some(mesh) = self.mesh.as_ref() {
                            let file_name = default_save_name(self.mesh_name.as_deref(), MshSaveFormat::V4);
                            request_save_dialog(mesh.clone(), file_name, MshSaveFormat::V4);
                        }
                        ui.close_menu();
                    }
                    if ui
                        .add_enabled(self.mesh.is_some(), egui::Button::new("Save As MSH 2.2..."))
                        .clicked()
                    {
                        if let Some(mesh) = self.mesh.as_ref() {
                            let file_name = default_save_name(self.mesh_name.as_deref(), MshSaveFormat::V2);
                            request_save_dialog(mesh.clone(), file_name, MshSaveFormat::V2);
                        }
                        ui.close_menu();
                    }
                    if ui.button("Quit").clicked() {
                        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    }
                });
            });
        });

        // Left panel — display controls
        egui::SidePanel::left("controls_panel")
            .default_width(200.0)
            .show(ctx, |ui| {
                ui.heading("Display");
                ui.separator();

                ui.checkbox(&mut self.config.show_nodes, "Show Nodes");
                ui.checkbox(&mut self.config.show_edges, "Show Edges");
                ui.checkbox(&mut self.config.show_faces, "Show Faces");
                ui.checkbox(&mut self.config.show_volumes, "Show Volumes");
                ui.separator();
                ui.checkbox(&mut self.config.show_gizmo, "Show Axes Gizmo");
                ui.separator();

                ui.label("Surface Opacity");
                ui.add(egui::Slider::new(&mut self.config.surface_opacity, 0.0..=1.0));

                ui.separator();
                if let Some(ref mesh) = self.mesh {
                    ui.label(format!("Nodes: {}", mesh.node_count()));
                    ui.label(format!("Elements: {}", mesh.element_count()));

                    let dim3 = mesh.elements_by_dimension(3).len();
                    let dim2 = mesh.elements_by_dimension(2).len();
                    let dim1 = mesh.elements_by_dimension(1).len();
                    let dim0 = mesh.elements_by_dimension(0).len();
                    if dim3 > 0 { ui.label(format!("  Volume: {}", dim3)); }
                    if dim2 > 0 { ui.label(format!("  Surface: {}", dim2)); }
                    if dim1 > 0 { ui.label(format!("  Edge: {}", dim1)); }
                    if dim0 > 0 { ui.label(format!("  Point: {}", dim0)); }
                } else {
                    ui.label("No mesh loaded");
                    ui.label("Drag & drop a .msh file");
                }
            });

        // Status bar
        egui::TopBottomPanel::bottom("status_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(&self.mesh_info);
            });
        });

        // Central panel — 3D viewport
        egui::CentralPanel::default().show(ctx, |ui| {
            let available = ui.available_size();
            let (rect, response) = ui.allocate_exact_size(available, egui::Sense::click_and_drag());

            // Handle mouse input for camera control
            if let Some(ref render_state) = self.render_state {
                let needs_repaint = handle_camera_input(render_state, &response, ui);
                if needs_repaint {
                    ctx.request_repaint();
                }
            }

            // Queue custom wgpu rendering
            let cb = egui_wgpu::Callback::new_paint_callback(
                rect,
                ViewportCallback,
            );
            ui.painter().add(cb);
        });
    }
}

/// Handle mouse input and update the camera in the Scene.
fn handle_camera_input(
    render_state: &egui_wgpu::RenderState,
    response: &egui::Response,
    ui: &egui::Ui,
) -> bool {

    let mut needs_repaint = false;

    // Rotation — left mouse drag
    if response.dragged_by(egui::PointerButton::Primary) {
        let delta = response.drag_delta();
        let mut renderer = render_state.renderer.write();
        if let Some(scene) = renderer.callback_resources.get_mut::<Scene>() {
            scene.camera.rotate(delta.x * 0.005, delta.y * 0.005);
            needs_repaint = true;
        }
    }

    // Pan — right mouse drag or middle mouse drag
    if response.dragged_by(egui::PointerButton::Secondary)
        || response.dragged_by(egui::PointerButton::Middle)
    {
        let delta = response.drag_delta();
        let mut renderer = render_state.renderer.write();
        if let Some(scene) = renderer.callback_resources.get_mut::<Scene>() {
            scene.camera.pan(delta.x, delta.y);
            needs_repaint = true;
        }
    }

    // Zoom — scroll wheel
    if response.hovered() {
        let scroll = ui.input(|i| i.smooth_scroll_delta.y);
        if scroll != 0.0 {
            let mut renderer = render_state.renderer.write();
            if let Some(scene) = renderer.callback_resources.get_mut::<Scene>() {
                scene.camera.zoom(scroll * 0.01);
                needs_repaint = true;
            }
        }
    }

    needs_repaint
}
