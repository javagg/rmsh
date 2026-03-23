use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::Path;
use std::path::PathBuf;
use std::thread;

use eframe::egui_wgpu;
use rmsh_algo::{MeshParams, Mesher3D, Polygon2D};
use rmsh_geo::extract::{PointData, SurfaceData, WireframeData};
use rmsh_model::{Mesh, Point3, Topology, Vector3, GSelection};
use rmsh_renderer::{RenderConfig, Scene};

use crate::io::{
    default_save_name, drain_io_events, enqueue_event, new_io_queue, request_open_dialog, request_open_path,
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
    /// Classified geometric model.
    topology: Option<Topology>,
    /// Currently selected geometric entity.
    topo_selection: Option<GSelection>,
    /// Whether the highlight GPU data needs to be re-uploaded.
    highlight_dirty: bool,
    /// Dihedral angle threshold for geometric classification (degrees).
    angle_threshold_deg: f64,
    /// Recently opened file paths (most recent first, capped at 10).
    recent_files: Vec<PathBuf>,
    /// Whether the currently loaded mesh came from a STEP file.
    source_is_step: bool,
    /// Target edge length for 2D meshing.
    meshing_size: f64,
    /// Target edge length for 3D meshing.
    meshing_size_3d: f64,
    /// Whether a background meshing task is running.
    meshing_in_progress: bool,
    /// Current meshing progress [0, 1].
    meshing_progress: f32,
    /// Meshing status line.
    meshing_message: String,
    /// Selected 3D meshing algorithm.
    meshing_algo_3d: MeshingAlgo3D,
    /// Hidden geometric region IDs.
    hidden_regions: HashSet<usize>,
    /// Hidden geometric face IDs.
    hidden_faces: HashSet<usize>,
    /// Hidden geometric edge IDs.
    hidden_edges: HashSet<usize>,
    /// Hidden geometric vertex IDs.
    hidden_vertices: HashSet<usize>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MeshingAlgo3D {
    CentroidStar,
    Delaunay,
}

impl MeshingAlgo3D {
    fn label(self) -> &'static str {
        match self {
            Self::CentroidStar => "Centroid-Star (stable)",
            Self::Delaunay => "Delaunay3D (experimental)",
        }
    }
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

        // Load recent files from persistent storage (newline-separated paths).
        let recent_files = cc
            .storage
            .and_then(|s| s.get_string("recent_files"))
            .map(|s| {
                s.lines()
                    .filter(|l| !l.is_empty())
                    .map(PathBuf::from)
                    .collect::<Vec<_>>()
            })
            .unwrap_or_default();

        Self {
            mesh: None,
            mesh_name: None,
            config: RenderConfig::default(),
            mesh_info: String::new(),
            scene_initialized: false,
            render_state,
            io_queue: new_io_queue(),
            topology: None,
            topo_selection: None,
            highlight_dirty: false,
            angle_threshold_deg: 40.0,
            recent_files,
            source_is_step: false,
            meshing_size: 0.25,
            meshing_size_3d: 0.25,
            meshing_in_progress: false,
            meshing_progress: 0.0,
            meshing_message: String::new(),
            meshing_algo_3d: MeshingAlgo3D::CentroidStar,
            hidden_regions: HashSet::new(),
            hidden_faces: HashSet::new(),
            hidden_edges: HashSet::new(),
            hidden_vertices: HashSet::new(),
        }
    }

    /// Add a path to the front of the recent-files list (dedup, max 10).
    fn push_recent(&mut self, path: PathBuf) {
        self.recent_files.retain(|p| p != &path);
        self.recent_files.insert(0, path);
        self.recent_files.truncate(10);
    }

    fn apply_loaded_mesh(&mut self, file_name: &str, data: &[u8], path: Option<PathBuf>) -> anyhow::Result<()> {
        let ext = path
            .as_ref()
            .and_then(|p| p.extension())
            .and_then(|e| e.to_str())
            .map(|e| e.to_ascii_lowercase())
            .or_else(|| {
                Path::new(file_name)
                    .extension()
                    .and_then(|e| e.to_str())
                    .map(|e| e.to_ascii_lowercase())
            });

        self.source_is_step = matches!(ext.as_deref(), Some("step") | Some("stp"));

        let mesh = match ext.as_deref() {
            Some("msh") => rmsh_io::load_msh_from_bytes(data)
                .map_err(anyhow::Error::from)?,
            Some("step") | Some("stp") => rmsh_io::load_step_from_bytes(data)
                .map_err(anyhow::Error::from)?,
            _ => rmsh_io::load_msh_from_bytes(data)
                .map_err(anyhow::Error::from)
                .or_else(|_| rmsh_io::load_step_from_bytes(data).map_err(anyhow::Error::from))?,
        };
        self.mesh_info = format!(
            "Nodes: {}  Elements: {}  File: {}",
            mesh.node_count(),
            mesh.element_count(),
            file_name
        );

        // Classify topology
        let topo = rmsh_geo::classify::classify(&mesh, self.angle_threshold_deg);
        log::info!(
            "Topology: {} regions, {} faces, {} edges, {} vertices",
            topo.regions.len(),
            topo.faces.len(),
            topo.edges.len(),
            topo.vertices.len(),
        );
        self.topology = Some(topo);
        self.topo_selection = None;
        self.highlight_dirty = true;
        self.hidden_regions.clear();
        self.hidden_faces.clear();
        self.hidden_edges.clear();
        self.hidden_vertices.clear();

        self.mesh = Some(mesh);
        self.mesh_name = Some(file_name.to_string());
        self.scene_initialized = false;
        if let Some(p) = path {
            self.push_recent(p);
        }
        Ok(())
    }

    fn apply_generated_mesh(&mut self, mesh: Mesh, mesh_name: String) {
        self.mesh_info = format!(
            "Nodes: {}  Elements: {}  File: {}",
            mesh.node_count(),
            mesh.element_count(),
            mesh_name
        );

        let topo = rmsh_geo::classify::classify(&mesh, self.angle_threshold_deg);
        self.topology = Some(topo);
        self.topo_selection = None;
        self.highlight_dirty = true;
        self.hidden_regions.clear();
        self.hidden_faces.clear();
        self.hidden_edges.clear();
        self.hidden_vertices.clear();

        self.mesh = Some(mesh);
        self.mesh_name = Some(mesh_name);
        self.source_is_step = false;
        self.scene_initialized = false;
    }

    fn start_2d_meshing(&mut self, ctx: &egui::Context) {
        if self.meshing_in_progress {
            return;
        }

        let Some(mesh) = self.mesh.clone() else {
            return;
        };
        let Some(topo) = self.topology.clone() else {
            return;
        };
        let Some(GSelection::Face(face_id)) = self.topo_selection else {
            return;
        };

        let mesh_size = self.meshing_size;
        let queue = self.io_queue.clone();
        let egui_ctx = ctx.clone();

        self.meshing_in_progress = true;
        self.meshing_progress = 0.0;
        self.meshing_message = "Preparing 2D meshing".to_string();

        thread::spawn(move || {
            enqueue_event(
                &queue,
                IoEvent::MeshingStarted {
                    message: format!("Start meshing face {}", face_id),
                },
            );
            egui_ctx.request_repaint();

            let mut report = |progress: f32, message: &str| {
                enqueue_event(
                    &queue,
                    IoEvent::MeshingProgress {
                        progress,
                        message: message.to_string(),
                    },
                );
                egui_ctx.request_repaint();
            };

            match mesh_face_async(&mesh, &topo, face_id, mesh_size, &mut report) {
                Ok(generated) => {
                    enqueue_event(
                        &queue,
                        IoEvent::MeshGenerated {
                            mesh: generated,
                            mesh_name: format!("meshed_face_{}.msh", face_id),
                        },
                    );
                }
                Err(err) => {
                    enqueue_event(&queue, IoEvent::Error(err));
                }
            }
            egui_ctx.request_repaint();
        });
    }

    fn start_3d_meshing(&mut self, ctx: &egui::Context) {
        if self.meshing_in_progress {
            return;
        }

        let Some(mesh) = self.mesh.clone() else {
            return;
        };
        let algo = self.meshing_algo_3d;
        let mesh_size_3d = self.meshing_size_3d;

        let queue = self.io_queue.clone();
        let egui_ctx = ctx.clone();

        self.meshing_in_progress = true;
        self.meshing_progress = 0.0;
        self.meshing_message = "Preparing 3D meshing".to_string();

        thread::spawn(move || {
            enqueue_event(
                &queue,
                IoEvent::MeshingStarted {
                    message: format!("Start 3D tetrahedralization ({})", algo.label()),
                },
            );
            egui_ctx.request_repaint();

            enqueue_event(
                &queue,
                IoEvent::MeshingProgress {
                    progress: 0.35,
                    message: format!(
                        "Building tetrahedra with {} (target size {:.4})",
                        algo.label(),
                        mesh_size_3d
                    ),
                },
            );
            egui_ctx.request_repaint();

            let params = MeshParams::with_size(mesh_size_3d);
            let result = match algo {
                MeshingAlgo3D::CentroidStar => {
                    let mesher = rmsh_algo::CentroidStarMesher3D;
                    mesher.mesh_3d(&mesh, &params)
                }
                MeshingAlgo3D::Delaunay => {
                    let mesher = rmsh_algo::Delaunay3D::default();
                    mesher.mesh_3d(&mesh, &params)
                }
            };

            match result {
                Ok(generated) => {
                    enqueue_event(
                        &queue,
                        IoEvent::MeshingProgress {
                            progress: 0.9,
                            message: "Finalizing generated 3D mesh".to_string(),
                        },
                    );
                    enqueue_event(
                        &queue,
                        IoEvent::MeshGenerated {
                            mesh: generated,
                            mesh_name: format!(
                                "meshed_volume_3d_{}.msh",
                                match algo {
                                    MeshingAlgo3D::CentroidStar => "centroid_star",
                                    MeshingAlgo3D::Delaunay => "delaunay",
                                }
                            ),
                        },
                    );
                }
                Err(err) => {
                    enqueue_event(&queue, IoEvent::Error(err.to_string()));
                }
            }

            egui_ctx.request_repaint();
        });
    }

    fn has_visibility_overrides(&self) -> bool {
        !self.hidden_regions.is_empty()
            || !self.hidden_faces.is_empty()
            || !self.hidden_edges.is_empty()
            || !self.hidden_vertices.is_empty()
    }

    fn extract_visible_geometry(&self, mesh: &Mesh, topo: &Topology) -> (SurfaceData, WireframeData, PointData) {
        // Region -> faces ownership map
        let mut face_to_regions: HashMap<usize, Vec<usize>> = HashMap::new();
        for region in &topo.regions {
            for fid in &region.face_ids {
                face_to_regions.entry(*fid).or_default().push(region.id);
            }
        }

        // Face -> edges ownership map
        let mut edge_to_faces: HashMap<usize, Vec<usize>> = HashMap::new();
        for face in &topo.faces {
            for eid in &face.edge_ids {
                edge_to_faces.entry(*eid).or_default().push(face.id);
            }
        }

        // Node -> geometric vertex lookup
        let node_to_vertex: HashMap<u64, usize> =
            topo.vertices.iter().map(|v| (v.node_id, v.id)).collect();

        let visible_regions: HashSet<usize> = topo
            .regions
            .iter()
            .filter(|r| !self.hidden_regions.contains(&r.id))
            .map(|r| r.id)
            .collect();

        let visible_faces: HashSet<usize> = topo
            .faces
            .iter()
            .filter(|f| {
                if self.hidden_faces.contains(&f.id) {
                    return false;
                }
                match face_to_regions.get(&f.id) {
                    Some(owners) if !owners.is_empty() => owners.iter().any(|rid| visible_regions.contains(rid)),
                    _ => true,
                }
            })
            .map(|f| f.id)
            .collect();

        let visible_edges: HashSet<usize> = topo
            .edges
            .iter()
            .filter(|e| {
                if self.hidden_edges.contains(&e.id) {
                    return false;
                }
                match edge_to_faces.get(&e.id) {
                    Some(owners) if !owners.is_empty() => owners.iter().any(|fid| visible_faces.contains(fid)),
                    _ => true,
                }
            })
            .map(|e| e.id)
            .collect();

        let visible_vertices: HashSet<usize> = topo
            .vertices
            .iter()
            .filter(|v| !self.hidden_vertices.contains(&v.id))
            .map(|v| v.id)
            .collect();

        let mut surface = SurfaceData {
            positions: Vec::new(),
            normals: Vec::new(),
            colors: Vec::new(),
            indices: Vec::new(),
        };

        for face in &topo.faces {
            if !visible_faces.contains(&face.id) {
                continue;
            }
            let color = [0.48, 0.62, 0.78];
            for poly in &face.mesh_faces {
                if poly.len() < 3 {
                    continue;
                }
                let pts: Vec<[f32; 3]> = poly
                    .iter()
                    .filter_map(|nid| mesh.nodes.get(nid))
                    .map(|n| [n.position.x as f32, n.position.y as f32, n.position.z as f32])
                    .collect();
                if pts.len() < 3 {
                    continue;
                }
                let normal = compute_normal(pts[0], pts[1], pts[2]);
                let base = surface.positions.len() as u32;
                for p in &pts {
                    surface.positions.push(*p);
                    surface.normals.push(normal);
                    surface.colors.push(color);
                }
                for i in 1..(pts.len() - 1) {
                    surface.indices.push(base);
                    surface.indices.push(base + i as u32);
                    surface.indices.push(base + i as u32 + 1);
                }
            }
        }

        let mut wireframe = WireframeData {
            positions: Vec::new(),
            indices: Vec::new(),
        };
        for edge in &topo.edges {
            if !visible_edges.contains(&edge.id) {
                continue;
            }
            for seg in edge.node_ids.windows(2) {
                let (Some(a), Some(b)) = (mesh.nodes.get(&seg[0]), mesh.nodes.get(&seg[1])) else {
                    continue;
                };
                let idx = wireframe.positions.len() as u32;
                wireframe
                    .positions
                    .push([a.position.x as f32, a.position.y as f32, a.position.z as f32]);
                wireframe
                    .positions
                    .push([b.position.x as f32, b.position.y as f32, b.position.z as f32]);
                wireframe.indices.push(idx);
                wireframe.indices.push(idx + 1);
            }
        }

        let mut points = PointData { positions: Vec::new() };
        for vertex in &topo.vertices {
            if !visible_vertices.contains(&vertex.id) {
                continue;
            }
            // If this vertex is attached to edges, require at least one visible owner edge.
            let mut has_visible_owner = false;
            for edge in &topo.edges {
                if !visible_edges.contains(&edge.id) {
                    continue;
                }
                let mut endpoints: Vec<usize> = edge.vertex_ids.iter().filter_map(|v| *v).collect();
                if endpoints.is_empty() {
                    if let Some(first) = edge.node_ids.first().and_then(|nid| node_to_vertex.get(nid)) {
                        endpoints.push(*first);
                    }
                    if let Some(last) = edge.node_ids.last().and_then(|nid| node_to_vertex.get(nid)) {
                        if !endpoints.contains(last) {
                            endpoints.push(*last);
                        }
                    }
                }
                if endpoints.contains(&vertex.id) {
                    has_visible_owner = true;
                    break;
                }
            }

            let is_orphan = !topo.edges.iter().any(|e| {
                e.vertex_ids.iter().flatten().any(|vid| *vid == vertex.id)
                    || e.node_ids.first().map(|nid| node_to_vertex.get(nid) == Some(&vertex.id)).unwrap_or(false)
                    || e.node_ids.last().map(|nid| node_to_vertex.get(nid) == Some(&vertex.id)).unwrap_or(false)
            });

            if has_visible_owner || is_orphan {
                if let Some(n) = mesh.nodes.get(&vertex.node_id) {
                    points
                        .positions
                        .push([n.position.x as f32, n.position.y as f32, n.position.z as f32]);
                }
            }
        }

        (surface, wireframe, points)
    }

    fn upload_mesh_to_gpu(&mut self, render_state: &egui_wgpu::RenderState) {
        if self.scene_initialized {
            return;
        }
        let Some(mesh) = &self.mesh else { return };

        let device = &render_state.device;

        // Extract geometry — use per-entity visibility filtering when toggles are active.
        let (surface, wireframe, points) = if let Some(ref topo) = self.topology {
            if self.has_visibility_overrides() {
                self.extract_visible_geometry(mesh, topo)
            } else {
                (
                    rmsh_geo::extract::extract_surface_colored(mesh, topo),
                    rmsh_geo::extract::extract_wireframe(mesh, &[1, 2, 3]),
                    rmsh_geo::extract::extract_points(mesh),
                )
            }
        } else {
            (
                rmsh_geo::extract::extract_surface(mesh),
                rmsh_geo::extract::extract_wireframe(mesh, &[1, 2, 3]),
                rmsh_geo::extract::extract_points(mesh),
            )
        };

        // Upload to GPU and fit camera
        let mut renderer = render_state.renderer.write();
        if let Some(scene) = renderer.callback_resources.get_mut::<Scene>() {
            scene.upload_mesh(device, &surface, &wireframe, &points);
            scene.clear_highlight();

            // Fit camera to mesh
            let center = mesh.center();
            let diag = mesh.diagonal_length() as f32;
            scene.camera.fit_to_bbox(
                [center.x as f32, center.y as f32, center.z as f32],
                diag,
            );
        }
        self.scene_initialized = true;
        self.highlight_dirty = true;
    }

    fn upload_highlight(&mut self, render_state: &egui_wgpu::RenderState) {
        if !self.highlight_dirty {
            return;
        }
        self.highlight_dirty = false;

        let device = &render_state.device;
        let mut renderer = render_state.renderer.write();
        let Some(scene) = renderer.callback_resources.get_mut::<Scene>() else {
            return;
        };

        let Some(mesh) = &self.mesh else {
            scene.clear_highlight();
            return;
        };

        let Some(topo) = &self.topology else {
            scene.clear_highlight();
            return;
        };

        let Some(sel) = &self.topo_selection else {
            scene.clear_highlight();
            return;
        };

        let (surface, wireframe) = rmsh_geo::extract::extract_highlight(mesh, topo, sel);
        scene.upload_highlight(device, surface.as_ref(), wireframe.as_ref());
    }
}

impl eframe::App for RmshApp {
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        let value = self
            .recent_files
            .iter()
            .map(|p| p.to_string_lossy().replace('\n', ""))
            .collect::<Vec<_>>()
            .join("\n");
        storage.set_string("recent_files", value);
    }

    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        for event in drain_io_events(&self.io_queue) {
            match event {
                IoEvent::MeshLoaded { file_name, data, path } => {
                    match self.apply_loaded_mesh(&file_name, &data, path) {
                        Ok(()) => log::info!("Loaded mesh: {}", file_name),
                        Err(e) => log::error!("Failed to load mesh: {}", e),
                    }
                }
                IoEvent::MeshGenerated { mesh, mesh_name } => {
                    self.apply_generated_mesh(mesh, mesh_name.clone());
                    self.meshing_in_progress = false;
                    self.meshing_progress = 1.0;
                    self.meshing_message = format!("Meshing finished: {}", mesh_name);
                    log::info!("{}", self.meshing_message);
                }
                IoEvent::MeshingStarted { message } => {
                    self.meshing_in_progress = true;
                    self.meshing_progress = 0.0;
                    self.meshing_message = message;
                }
                IoEvent::MeshingProgress { progress, message } => {
                    self.meshing_progress = progress.clamp(0.0, 1.0);
                    self.meshing_message = message;
                }
                IoEvent::Error(message) => {
                    if self.meshing_in_progress {
                        self.meshing_in_progress = false;
                        self.meshing_message = format!("Meshing failed: {}", message);
                    }
                    log::error!("{}", message);
                }
            }
        }

        // Upload mesh to GPU if needed
        if let Some(render_state) = self.render_state.clone() {
            self.upload_mesh_to_gpu(&render_state);
            self.upload_highlight(&render_state);

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
                    if ui.button("Open...").clicked() {
                        request_open_dialog(self.io_queue.clone(), ctx.clone());
                        ui.close_menu();
                    }
                    // Open Recent submenu
                    let recent_files_snapshot = self.recent_files.clone();
                    ui.menu_button("Open Recent...", |ui| {
                        if recent_files_snapshot.is_empty() {
                            ui.add_enabled(false, egui::Label::new("(No recent files)"));
                        } else {
                            for path in &recent_files_snapshot {
                                let label = path
                                    .file_name()
                                    .unwrap_or_default()
                                    .to_string_lossy()
                                    .to_string();
                                let response = ui
                                    .add(egui::Button::new(&label))
                                    .on_hover_text(path.to_string_lossy().as_ref());
                                if response.clicked() {
                                    request_open_path(
                                        path.clone(),
                                        self.io_queue.clone(),
                                        ctx.clone(),
                                    );
                                    ui.close_menu();
                                }
                            }
                            ui.separator();
                            if ui.button("Clear Recent Files").clicked() {
                                self.recent_files.clear();
                                ui.close_menu();
                            }
                        }
                    });
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

                ui.menu_button("Meshing", |ui| {
                    ui.menu_button("2D Meshing", |ui| {
                        ui.label("Target edge length");
                        ui.add(
                            egui::DragValue::new(&mut self.meshing_size)
                                .range(0.001..=1.0e6)
                                .speed(0.01),
                        );

                        ui.separator();
                        let is_face_selected = matches!(self.topo_selection, Some(GSelection::Face(_)));
                        if !self.source_is_step {
                            ui.small("Load a STEP model to enable 2D meshing.");
                        } else if !is_face_selected {
                            ui.small("Select one face in the Topology panel first.");
                        }

                        let can_start = self.source_is_step
                            && is_face_selected
                            && !self.meshing_in_progress
                            && self.meshing_size > 0.0;

                        if ui
                            .add_enabled(can_start, egui::Button::new("Triangulate Selected Face"))
                            .clicked()
                        {
                            self.start_2d_meshing(ctx);
                            ui.close_menu();
                        }

                        if self.meshing_in_progress {
                            ui.separator();
                            ui.add(
                                egui::ProgressBar::new(self.meshing_progress)
                                    .show_percentage()
                                    .text(&self.meshing_message),
                            );
                        }
                    });

                    ui.menu_button("3D Meshing", |ui| {
                        ui.small("Generate tetrahedral volume mesh from closed surface.");
                        ui.separator();
                        ui.label("Algorithm");
                        ui.radio_value(
                            &mut self.meshing_algo_3d,
                            MeshingAlgo3D::CentroidStar,
                            MeshingAlgo3D::CentroidStar.label(),
                        );
                        ui.radio_value(
                            &mut self.meshing_algo_3d,
                            MeshingAlgo3D::Delaunay,
                            MeshingAlgo3D::Delaunay.label(),
                        );
                        ui.separator();
                        ui.label("Target edge length");
                        ui.add(
                            egui::DragValue::new(&mut self.meshing_size_3d)
                                .range(0.001..=1.0e6)
                                .speed(0.01),
                        );

                        let can_start = self.source_is_step && !self.meshing_in_progress;
                        if !self.source_is_step {
                            ui.small("Load a STEP model to enable 3D meshing.");
                        }

                        if ui
                            .add_enabled(can_start, egui::Button::new("Tetrahedralize Model"))
                            .clicked()
                        {
                            self.start_3d_meshing(ctx);
                            ui.close_menu();
                        }

                        if self.meshing_in_progress {
                            ui.separator();
                            ui.add(
                                egui::ProgressBar::new(self.meshing_progress)
                                    .show_percentage()
                                    .text(&self.meshing_message),
                            );
                        }
                    });
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

                if ui.button("Isometric View").clicked() {
                    if let Some(ref render_state) = self.render_state {
                        let mut renderer = render_state.renderer.write();
                        if let Some(scene) = renderer.callback_resources.get_mut::<Scene>() {
                            scene.camera.set_isometric();
                        }
                    }
                }

                // Projection mode toggle
                let proj_label = {
                    if let Some(ref render_state) = self.render_state {
                        let renderer = render_state.renderer.read();
                        if let Some(scene) = renderer.callback_resources.get::<Scene>() {
                            if scene.camera.orthographic { "Perspective" } else { "Orthographic" }
                        } else { "Orthographic" }
                    } else { "Orthographic" }
                };
                if ui.button(proj_label).clicked() {
                    if let Some(ref render_state) = self.render_state {
                        let mut renderer = render_state.renderer.write();
                        if let Some(scene) = renderer.callback_resources.get_mut::<Scene>() {
                            scene.camera.toggle_projection();
                        }
                    }
                }
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

        // Right panel — topology tree
        let mut reclassify = false;
        let mut visibility_changed = false;
        egui::SidePanel::right("topology_panel")
            .default_width(220.0)
            .show(ctx, |ui| {
                ui.heading("Topology");
                ui.separator();

                if self.topology.is_some() {
                    // Angle threshold control
                    let mut threshold = self.angle_threshold_deg;
                    ui.horizontal(|ui| {
                        ui.label("Angle °");
                        if ui
                            .add(egui::DragValue::new(&mut threshold).range(1.0..=180.0).speed(1.0))
                            .changed()
                        {
                            self.angle_threshold_deg = threshold;
                            reclassify = true;
                        }
                    });
                    ui.separator();

                    // Clear selection button
                    if self.topo_selection.is_some() {
                        if ui.small_button("Clear Selection").clicked() {
                            self.topo_selection = None;
                            self.highlight_dirty = true;
                        }
                        ui.separator();
                    }
                }

                if let Some(ref topo) = self.topology {
                    // Summary
                    ui.label(format!(
                        "V:{} F:{} E:{} P:{}",
                        topo.regions.len(),
                        topo.faces.len(),
                        topo.edges.len(),
                        topo.vertices.len(),
                    ));
                    ui.separator();

                    egui::CollapsingHeader::new("Visibility")
                        .id_salt("topo_visibility")
                        .default_open(false)
                        .show(ui, |ui| {
                            ui.label("Toggle geometric entities:");

                            egui::CollapsingHeader::new(format!("Regions ({})", topo.regions.len()))
                                .id_salt("vis_regions")
                                .default_open(false)
                                .show(ui, |ui| {
                                    for region in &topo.regions {
                                        let mut visible = !self.hidden_regions.contains(&region.id);
                                        if ui.checkbox(&mut visible, format!("Region {}", region.id)).changed() {
                                            if visible {
                                                self.hidden_regions.remove(&region.id);
                                            } else {
                                                self.hidden_regions.insert(region.id);
                                            }
                                            visibility_changed = true;
                                        }
                                    }
                                });

                            egui::CollapsingHeader::new(format!("Faces ({})", topo.faces.len()))
                                .id_salt("vis_faces")
                                .default_open(false)
                                .show(ui, |ui| {
                                    for face in &topo.faces {
                                        let mut visible = !self.hidden_faces.contains(&face.id);
                                        if ui.checkbox(&mut visible, format!("Face {}", face.id)).changed() {
                                            if visible {
                                                self.hidden_faces.remove(&face.id);
                                            } else {
                                                self.hidden_faces.insert(face.id);
                                            }
                                            visibility_changed = true;
                                        }
                                    }
                                });

                            egui::CollapsingHeader::new(format!("Edges ({})", topo.edges.len()))
                                .id_salt("vis_edges")
                                .default_open(false)
                                .show(ui, |ui| {
                                    for edge in &topo.edges {
                                        let mut visible = !self.hidden_edges.contains(&edge.id);
                                        if ui.checkbox(&mut visible, format!("Edge {}", edge.id)).changed() {
                                            if visible {
                                                self.hidden_edges.remove(&edge.id);
                                            } else {
                                                self.hidden_edges.insert(edge.id);
                                            }
                                            visibility_changed = true;
                                        }
                                    }
                                });

                            egui::CollapsingHeader::new(format!("Vertices ({})", topo.vertices.len()))
                                .id_salt("vis_vertices")
                                .default_open(false)
                                .show(ui, |ui| {
                                    for vertex in &topo.vertices {
                                        let mut visible = !self.hidden_vertices.contains(&vertex.id);
                                        if ui.checkbox(&mut visible, format!("Vertex {}", vertex.id)).changed() {
                                            if visible {
                                                self.hidden_vertices.remove(&vertex.id);
                                            } else {
                                                self.hidden_vertices.insert(vertex.id);
                                            }
                                            visibility_changed = true;
                                        }
                                    }
                                });

                            if ui.small_button("Show All").clicked() {
                                self.hidden_regions.clear();
                                self.hidden_faces.clear();
                                self.hidden_edges.clear();
                                self.hidden_vertices.clear();
                                visibility_changed = true;
                            }
                        });

                    ui.separator();

                    // Tree view (Volume -> Face -> Edge -> Vertex)
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        // Clone data we need so we don't borrow self immutably during UI interaction.
                        let volumes = topo.regions.clone();
                        let faces = topo.faces.clone();
                        let edges = topo.edges.clone();
                        let vertices = topo.vertices.clone();

                        let face_map: HashMap<usize, _> =
                            faces.iter().cloned().map(|f| (f.id, f)).collect();
                        let edge_map: HashMap<usize, _> =
                            edges.iter().cloned().map(|e| (e.id, e)).collect();
                        let vertex_map: HashMap<usize, _> =
                            vertices.iter().cloned().map(|v| (v.id, v)).collect();
                        let node_to_vertex: HashMap<u64, usize> =
                            vertices.iter().map(|v| (v.node_id, v.id)).collect();

                        let mut new_selection = self.topo_selection;

                        let mut used_faces: HashSet<usize> = HashSet::new();
                        let mut used_edges: HashSet<usize> = HashSet::new();
                        let mut used_vertices: HashSet<usize> = HashSet::new();

                        if !volumes.is_empty() {
                            egui::CollapsingHeader::new(format!("Volumes ({})", volumes.len()))
                                .id_salt("topo_volumes_tree")
                                .default_open(true)
                                .show(ui, |ui| {
                                    for vol in &volumes {
                                        let header = egui::CollapsingHeader::new(format!(
                                            "Volume {} ({} elems, {} faces)",
                                            vol.id,
                                            vol.element_ids.len(),
                                            vol.face_ids.len()
                                        ))
                                        .id_salt(("vol", vol.id))
                                        .default_open(false)
                                        .show(ui, |ui| {
                                            for fid in &vol.face_ids {
                                                used_faces.insert(*fid);
                                                let Some(face) = face_map.get(fid) else {
                                                    continue;
                                                };

                                                let face_header = egui::CollapsingHeader::new(format!(
                                                    "Face {} ({} mesh faces, {} edges)",
                                                    face.id,
                                                    face.mesh_faces.len(),
                                                    face.edge_ids.len()
                                                ))
                                                .id_salt(("face", vol.id, face.id))
                                                .default_open(false)
                                                .show(ui, |ui| {
                                                    for eid in &face.edge_ids {
                                                        used_edges.insert(*eid);
                                                        let Some(edge) = edge_map.get(eid) else {
                                                            continue;
                                                        };

                                                        let mut vids: Vec<usize> = edge
                                                            .vertex_ids
                                                            .iter()
                                                            .filter_map(|v| *v)
                                                            .collect();
                                                        if vids.is_empty() {
                                                            if let Some(first) = edge.node_ids.first() {
                                                                if let Some(vid) = node_to_vertex.get(first) {
                                                                    vids.push(*vid);
                                                                }
                                                            }
                                                            if let Some(last) = edge.node_ids.last() {
                                                                if let Some(vid) = node_to_vertex.get(last) {
                                                                    if !vids.contains(vid) {
                                                                        vids.push(*vid);
                                                                    }
                                                                }
                                                            }
                                                        }
                                                        for vid in &vids {
                                                            used_vertices.insert(*vid);
                                                        }

                                                        let edge_header = egui::CollapsingHeader::new(format!(
                                                            "Edge {} ({} nodes, {} vertices)",
                                                            edge.id,
                                                            edge.node_ids.len(),
                                                            vids.len()
                                                        ))
                                                        .id_salt(("edge", vol.id, face.id, edge.id))
                                                        .default_open(false)
                                                        .show(ui, |ui| {
                                                            for vid in vids {
                                                                if let Some(vertex) = vertex_map.get(&vid) {
                                                                    let selected = new_selection
                                                                        == Some(GSelection::Vertex(vertex.id));
                                                                    if ui
                                                                        .selectable_label(
                                                                            selected,
                                                                            format!(
                                                                                "Vertex {} (node {})",
                                                                                vertex.id, vertex.node_id
                                                                            ),
                                                                        )
                                                                        .clicked()
                                                                    {
                                                                        toggle_topo_selection(
                                                                            &mut new_selection,
                                                                            GSelection::Vertex(vertex.id),
                                                                        );
                                                                    }
                                                                }
                                                            }
                                                        });

                                                        if edge_header.header_response.clicked() {
                                                            toggle_topo_selection(
                                                                &mut new_selection,
                                                                GSelection::Edge(edge.id),
                                                            );
                                                        }
                                                    }
                                                });

                                                if face_header.header_response.clicked() {
                                                    toggle_topo_selection(
                                                        &mut new_selection,
                                                        GSelection::Face(face.id),
                                                    );
                                                }
                                            }
                                        });

                                        if header.header_response.clicked() {
                                            toggle_topo_selection(
                                                &mut new_selection,
                                                GSelection::Region(vol.id),
                                            );
                                        }
                                    }
                                });
                        }

                        // Orphan faces not referenced by any volume (e.g. pure surface meshes).
                        let orphan_faces: Vec<_> = faces
                            .iter()
                            .filter(|f| !used_faces.contains(&f.id))
                            .cloned()
                            .collect();
                        if !orphan_faces.is_empty() {
                            egui::CollapsingHeader::new(format!("Faces ({})", orphan_faces.len()))
                                .id_salt("topo_orphan_faces")
                                .default_open(volumes.is_empty())
                                .show(ui, |ui| {
                                    for face in &orphan_faces {
                                        let face_header = egui::CollapsingHeader::new(format!(
                                            "Face {} ({} mesh faces, {} edges)",
                                            face.id,
                                            face.mesh_faces.len(),
                                            face.edge_ids.len()
                                        ))
                                        .id_salt(("orphan_face", face.id))
                                        .default_open(false)
                                        .show(ui, |ui| {
                                            for eid in &face.edge_ids {
                                                used_edges.insert(*eid);
                                                let Some(edge) = edge_map.get(eid) else {
                                                    continue;
                                                };
                                                let selected =
                                                    new_selection == Some(GSelection::Edge(edge.id));
                                                if ui
                                                    .selectable_label(
                                                        selected,
                                                        format!(
                                                            "Edge {} ({} nodes)",
                                                            edge.id,
                                                            edge.node_ids.len()
                                                        ),
                                                    )
                                                    .clicked()
                                                {
                                                    toggle_topo_selection(
                                                        &mut new_selection,
                                                        GSelection::Edge(edge.id),
                                                    );
                                                }
                                            }
                                        });
                                        if face_header.header_response.clicked() {
                                            toggle_topo_selection(
                                                &mut new_selection,
                                                GSelection::Face(face.id),
                                            );
                                        }
                                    }
                                });
                        }

                        // Orphan edges not referenced by any face.
                        let orphan_edges: Vec<_> = edges
                            .iter()
                            .filter(|e| !used_edges.contains(&e.id))
                            .cloned()
                            .collect();
                        if !orphan_edges.is_empty() {
                            egui::CollapsingHeader::new(format!("Edges ({})", orphan_edges.len()))
                                .id_salt("topo_orphan_edges")
                                .default_open(!orphan_faces.is_empty())
                                .show(ui, |ui| {
                                    for edge in &orphan_edges {
                                        let mut vids: Vec<usize> =
                                            edge.vertex_ids.iter().filter_map(|v| *v).collect();
                                        if vids.is_empty() {
                                            if let Some(first) = edge.node_ids.first() {
                                                if let Some(vid) = node_to_vertex.get(first) {
                                                    vids.push(*vid);
                                                }
                                            }
                                            if let Some(last) = edge.node_ids.last() {
                                                if let Some(vid) = node_to_vertex.get(last) {
                                                    if !vids.contains(vid) {
                                                        vids.push(*vid);
                                                    }
                                                }
                                            }
                                        }
                                        for vid in &vids {
                                            used_vertices.insert(*vid);
                                        }

                                        let edge_header = egui::CollapsingHeader::new(format!(
                                            "Edge {} ({} nodes, {} vertices)",
                                            edge.id,
                                            edge.node_ids.len(),
                                            vids.len()
                                        ))
                                        .id_salt(("orphan_edge", edge.id))
                                        .default_open(false)
                                        .show(ui, |ui| {
                                            for vid in vids {
                                                if let Some(vertex) = vertex_map.get(&vid) {
                                                    let selected = new_selection
                                                        == Some(GSelection::Vertex(vertex.id));
                                                    if ui
                                                        .selectable_label(
                                                            selected,
                                                            format!(
                                                                "Vertex {} (node {})",
                                                                vertex.id, vertex.node_id
                                                            ),
                                                        )
                                                        .clicked()
                                                    {
                                                        toggle_topo_selection(
                                                            &mut new_selection,
                                                            GSelection::Vertex(vertex.id),
                                                        );
                                                    }
                                                }
                                            }
                                        });
                                        if edge_header.header_response.clicked() {
                                            toggle_topo_selection(
                                                &mut new_selection,
                                                GSelection::Edge(edge.id),
                                            );
                                        }
                                    }
                                });
                        }

                        // Orphan vertices not referenced by any shown edge.
                        let orphan_vertices: Vec<_> = vertices
                            .iter()
                            .filter(|v| !used_vertices.contains(&v.id))
                            .cloned()
                            .collect();
                        if !orphan_vertices.is_empty() {
                            egui::CollapsingHeader::new(format!(
                                "Vertices ({})",
                                orphan_vertices.len()
                            ))
                            .id_salt("topo_orphan_vertices")
                            .default_open(!orphan_edges.is_empty())
                            .show(ui, |ui| {
                                for vertex in &orphan_vertices {
                                    let selected =
                                        new_selection == Some(GSelection::Vertex(vertex.id));
                                    if ui
                                        .selectable_label(
                                            selected,
                                            format!(
                                                "Vertex {} (node {})",
                                                vertex.id, vertex.node_id
                                            ),
                                        )
                                        .clicked()
                                    {
                                        toggle_topo_selection(
                                            &mut new_selection,
                                            GSelection::Vertex(vertex.id),
                                        );
                                    }
                                }
                            });
                        }

                        if new_selection != self.topo_selection {
                            self.topo_selection = new_selection;
                            self.highlight_dirty = true;
                        }
                    });
                } else {
                    ui.label("No topology");
                }
            });

        if visibility_changed {
            self.scene_initialized = false;
            self.highlight_dirty = true;
            if let Some(sel) = self.topo_selection {
                let hidden = match sel {
                    GSelection::Region(id) => self.hidden_regions.contains(&id),
                    GSelection::Face(id) => self.hidden_faces.contains(&id),
                    GSelection::Edge(id) => self.hidden_edges.contains(&id),
                    GSelection::Vertex(id) => self.hidden_vertices.contains(&id),
                };
                if hidden {
                    self.topo_selection = None;
                }
            }
        }

        // Re-classify topology if angle threshold changed
        if reclassify {
            if let Some(ref mesh) = self.mesh {
                let new_topo = rmsh_geo::classify::classify(mesh, self.angle_threshold_deg);
                self.topology = Some(new_topo);
                self.topo_selection = None;
                self.highlight_dirty = true;
                self.hidden_regions.clear();
                self.hidden_faces.clear();
                self.hidden_edges.clear();
                self.hidden_vertices.clear();
                self.scene_initialized = false;
            }
        }

        // Status bar
        egui::TopBottomPanel::bottom("status_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(&self.mesh_info);
                if self.meshing_in_progress || !self.meshing_message.is_empty() {
                    ui.separator();
                    ui.label(&self.meshing_message);
                    if self.meshing_in_progress {
                        ui.add(
                            egui::ProgressBar::new(self.meshing_progress)
                                .desired_width(140.0)
                                .show_percentage(),
                        );
                    }
                }
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

fn mesh_face_async(
    mesh: &Mesh,
    topo: &Topology,
    face_id: usize,
    mesh_size: f64,
    report: &mut dyn FnMut(f32, &str),
) -> Result<Mesh, String> {
    if mesh_size <= 0.0 {
        return Err("mesh_size must be positive".to_string());
    }

    report(0.1, "Preparing selected face");
    let face = topo
        .faces
        .iter()
        .find(|f| f.id == face_id)
        .ok_or_else(|| format!("Face {} not found in topology", face_id))?;

    if face.mesh_faces.is_empty() {
        return Err(format!("Face {} has no mesh polygons", face_id));
    }

    report(0.25, "Collecting face nodes");
    let mut face_node_ids: HashSet<u64> = HashSet::new();
    for poly in &face.mesh_faces {
        for nid in poly {
            face_node_ids.insert(*nid);
        }
    }
    if face_node_ids.len() < 3 {
        return Err(format!("Face {} has fewer than 3 unique nodes", face_id));
    }

    let mut node_points: Vec<(u64, Point3)> = Vec::with_capacity(face_node_ids.len());
    for nid in &face_node_ids {
        let node = mesh
            .nodes
            .get(nid)
            .ok_or_else(|| format!("Node {} not found in mesh", nid))?;
        node_points.push((*nid, node.position));
    }

    // Build a local 2D frame on the selected face plane.
    let p0 = node_points[0].1;
    let mut basis: Option<(Vector3, Vector3)> = None;
    for i in 1..node_points.len() {
        let u_try = node_points[i].1 - p0;
        if u_try.norm() < 1e-12 {
            continue;
        }
        for j in (i + 1)..node_points.len() {
            let v_try = node_points[j].1 - p0;
            let n = u_try.cross(&v_try);
            if n.norm() > 1e-10 {
                let u = u_try.normalize();
                let n_norm = n.normalize();
                let v = n_norm.cross(&u).normalize();
                basis = Some((u, v));
                break;
            }
        }
        if basis.is_some() {
            break;
        }
    }

    let (u_axis, v_axis) = basis.ok_or_else(|| {
        format!(
            "Face {} appears degenerate (cannot construct local plane basis)",
            face_id
        )
    })?;

    report(0.4, "Extracting boundary loop");
    let polygon = polygon_from_face(mesh, face, p0, u_axis, v_axis)?;

    report(0.65, "Running 2D triangulation");
    let mut generated = rmsh_algo::mesh_polygon(&polygon, mesh_size).map_err(|e| e.to_string())?;

    report(0.85, "Projecting 2D mesh back to 3D");
    for node in generated.nodes.values_mut() {
        let x = node.position.x;
        let y = node.position.y;
        let p3 = p0 + u_axis * x + v_axis * y;
        node.position = p3;
    }

    report(1.0, "Meshing complete");
    Ok(generated)
}

fn polygon_from_face(
    mesh: &Mesh,
    face: &rmsh_model::GFace,
    origin: Point3,
    u_axis: Vector3,
    v_axis: Vector3,
) -> Result<Polygon2D, String> {
    let mut edge_counts: BTreeMap<(u64, u64), usize> = BTreeMap::new();
    for poly in &face.mesh_faces {
        if poly.len() < 2 {
            continue;
        }
        for i in 0..poly.len() {
            let a = poly[i];
            let b = poly[(i + 1) % poly.len()];
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_counts.entry(key).or_insert(0) += 1;
        }
    }

    let boundary_edges: Vec<(u64, u64)> = edge_counts
        .iter()
        .filter_map(|(edge, count)| (*count == 1).then_some(*edge))
        .collect();
    if boundary_edges.is_empty() {
        return Err("Could not find a boundary loop for selected face".to_string());
    }

    let mut adjacency: HashMap<u64, Vec<u64>> = HashMap::new();
    for (a, b) in &boundary_edges {
        adjacency.entry(*a).or_default().push(*b);
        adjacency.entry(*b).or_default().push(*a);
    }

    if adjacency.values().any(|neighbors| neighbors.len() != 2) {
        return Err(
            "Selected face has non-manifold boundary; only single closed-loop faces are supported"
                .to_string(),
        );
    }

    let start = *adjacency
        .keys()
        .min()
        .ok_or_else(|| "Boundary loop is empty".to_string())?;
    let mut loop_nodes = vec![start];
    let mut prev: Option<u64> = None;
    let mut current = start;

    for _ in 0..=boundary_edges.len() {
        let neighbors = adjacency
            .get(&current)
            .ok_or_else(|| format!("Boundary node {} has no neighbors", current))?;
        let next = match prev {
            Some(p) => neighbors
                .iter()
                .copied()
                .find(|n| *n != p)
                .ok_or_else(|| format!("Boundary walk failed at node {}", current))?,
            None => neighbors[0],
        };

        if next == start {
            break;
        }

        loop_nodes.push(next);
        prev = Some(current);
        current = next;
    }

    let closes = adjacency
        .get(&current)
        .map(|neighbors| neighbors.contains(&start))
        .unwrap_or(false);
    if !closes {
        return Err("Failed to close face boundary loop".to_string());
    }

    if loop_nodes.len() < 3 {
        return Err("Boundary loop has fewer than 3 vertices".to_string());
    }

    let mut vertices = Vec::with_capacity(loop_nodes.len());
    for nid in &loop_nodes {
        let point = mesh
            .nodes
            .get(nid)
            .map(|n| n.position)
            .ok_or_else(|| format!("Boundary node {} not found in mesh", nid))?;
        let d = point - origin;
        vertices.push([d.dot(&u_axis), d.dot(&v_axis)]);
    }

    Ok(Polygon2D::new(vertices))
}

fn compute_normal(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> [f32; 3] {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let n = [
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    ];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len > 1e-10 {
        [n[0] / len, n[1] / len, n[2] / len]
    } else {
        [0.0, 0.0, 1.0]
    }
}

fn toggle_topo_selection(selection: &mut Option<GSelection>, target: GSelection) {
    if *selection == Some(target) {
        *selection = None;
    } else {
        *selection = Some(target);
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

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use rmsh_algo::{Delaunay3D, MeshParams, Mesher3D};

    #[test]
    fn viewer_step_to_3d_meshing_gmsh_roundtrip() {
        let step_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("..")
            .join("testdata")
            .join("my_cube.step");

        let step_bytes = std::fs::read(&step_path)
            .unwrap_or_else(|e| panic!("failed to read {}: {}", step_path.display(), e));

        // Simulate viewer load pipeline: STEP -> mesh -> topology classification.
        let step_mesh = rmsh_io::load_step_from_bytes(&step_bytes).expect("STEP parsing should succeed");
        let topo = rmsh_geo::classify::classify(&step_mesh, 40.0);
        assert!(!topo.faces.is_empty(), "classified topology should contain faces");

        // Simulate 3D meshing action from viewer menu.
        let volume_mesh = rmsh_algo::tetrahedralize_closed_surface(&step_mesh)
            .expect("3D meshing should succeed for cube STEP");
        assert!(
            volume_mesh.elements_by_dimension(3).len() > 0,
            "meshed result should contain 3D elements"
        );

        // Simulate viewer Save As Gmsh v4 and validate readback.
        let mut v4_bytes = Vec::new();
        rmsh_io::write_msh_v4(&mut v4_bytes, &volume_mesh).expect("MSH v4 write should succeed");
        let v4_loaded = rmsh_io::load_msh_from_bytes(&v4_bytes).expect("MSH v4 readback should succeed");
        assert_eq!(v4_loaded.node_count(), volume_mesh.node_count());
        assert_eq!(v4_loaded.element_count(), volume_mesh.element_count());

        // Simulate viewer Save As Gmsh v2 and validate readback.
        let mut v2_bytes = Vec::new();
        rmsh_io::write_msh_v2(&mut v2_bytes, &volume_mesh).expect("MSH v2 write should succeed");
        let v2_loaded = rmsh_io::load_msh_from_bytes(&v2_bytes).expect("MSH v2 readback should succeed");
        assert_eq!(v2_loaded.node_count(), volume_mesh.node_count());
        assert_eq!(v2_loaded.element_count(), volume_mesh.element_count());
    }

    #[test]
    fn viewer_step_to_3d_meshing_via_centroid_star_trait_roundtrip() {
        let step_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("..")
            .join("testdata")
            .join("my_cube.step");

        let step_bytes = std::fs::read(&step_path)
            .unwrap_or_else(|e| panic!("failed to read {}: {}", step_path.display(), e));

        let step_mesh = rmsh_io::load_step_from_bytes(&step_bytes).expect("STEP parsing should succeed");

        let params = MeshParams::with_size(0.25);
        let mesher = rmsh_algo::CentroidStarMesher3D;
        let volume_mesh = mesher
            .mesh_3d(&step_mesh, &params)
            .expect("Centroid-star meshing should succeed for cube STEP");
        assert!(volume_mesh.elements_by_dimension(3).len() > 0);

        let mut v4_bytes = Vec::new();
        rmsh_io::write_msh_v4(&mut v4_bytes, &volume_mesh).expect("MSH v4 write should succeed");
        let v4_loaded = rmsh_io::load_msh_from_bytes(&v4_bytes).expect("MSH v4 readback should succeed");
        assert_eq!(v4_loaded.node_count(), volume_mesh.node_count());
        assert_eq!(v4_loaded.element_count(), volume_mesh.element_count());
    }

    #[test]
    fn viewer_step_to_3d_meshing_via_delaunay_trait_roundtrip() {
        let step_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("..")
            .join("testdata")
            .join("my_cube.step");

        let step_bytes = std::fs::read(&step_path)
            .unwrap_or_else(|e| panic!("failed to read {}: {}", step_path.display(), e));

        let step_mesh = rmsh_io::load_step_from_bytes(&step_bytes).expect("STEP parsing should succeed");

        let params = MeshParams::with_size(0.25);
        let mesher = Delaunay3D::default();
        let volume_mesh = mesher
            .mesh_3d(&step_mesh, &params)
            .expect("Delaunay3D meshing should succeed for cube STEP");
        assert!(volume_mesh.elements_by_dimension(3).len() > 0);

        let mut v2_bytes = Vec::new();
        rmsh_io::write_msh_v2(&mut v2_bytes, &volume_mesh).expect("MSH v2 write should succeed");
        let v2_loaded = rmsh_io::load_msh_from_bytes(&v2_bytes).expect("MSH v2 readback should succeed");
        assert_eq!(v2_loaded.node_count(), volume_mesh.node_count());
        assert_eq!(v2_loaded.element_count(), volume_mesh.element_count());
    }

    #[test]
    fn viewer_step_to_3d_meshing_via_delaunay_respects_size() {
        let step_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("..")
            .join("testdata")
            .join("my_cube.step");

        let step_bytes = std::fs::read(&step_path)
            .unwrap_or_else(|e| panic!("failed to read {}: {}", step_path.display(), e));
        let step_mesh = rmsh_io::load_step_from_bytes(&step_bytes).expect("STEP parsing should succeed");

        let mut coarse = MeshParams::with_size(1.0);
        coarse.max_size = 1.2;
        coarse.optimize_passes = 2;

        let mut fine = MeshParams::with_size(0.25);
        fine.max_size = 0.3;
        fine.optimize_passes = 2;

        let mesher = Delaunay3D::default();
        let coarse_mesh = mesher
            .mesh_3d(&step_mesh, &coarse)
            .expect("coarse Delaunay3D meshing should succeed");
        let fine_mesh = mesher
            .mesh_3d(&step_mesh, &fine)
            .expect("fine Delaunay3D meshing should succeed");

        let coarse_tets = coarse_mesh.elements_by_dimension(3).len();
        let fine_tets = fine_mesh.elements_by_dimension(3).len();
        assert!(
            fine_tets > coarse_tets,
            "smaller size should create denser mesh: coarse={coarse_tets}, fine={fine_tets}"
        );
    }
}
