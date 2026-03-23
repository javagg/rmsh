use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use rmsh_model::Mesh;

pub type IoQueue = Arc<Mutex<VecDeque<IoEvent>>>;

#[derive(Debug)]
pub enum IoEvent {
    MeshLoaded { file_name: String, data: Vec<u8>, path: Option<PathBuf> },
    Error(String),
}

#[derive(Clone, Copy)]
pub enum MshSaveFormat {
    V2,
    V4,
}

pub fn new_io_queue() -> IoQueue {
    Arc::new(Mutex::new(VecDeque::new()))
}

pub fn drain_io_events(queue: &IoQueue) -> Vec<IoEvent> {
    match queue.lock() {
        Ok(mut queue) => queue.drain(..).collect(),
        Err(_) => vec![IoEvent::Error("IO queue lock poisoned".to_string())],
    }
}

pub fn request_open_dialog(queue: IoQueue, ctx: egui::Context) {
    spawn_dialog_task(async move {
        let file = rfd::AsyncFileDialog::new()
            .add_filter("Gmsh Mesh", &["msh"])
            .add_filter("STEP Model", &["step", "stp"])
            .add_filter("All Files", &["*"])
            .set_title("Open Mesh/STEP File")
            .pick_file()
            .await;

        if let Some(file) = file {
            let file_name = file.file_name();
            #[cfg(not(target_arch = "wasm32"))]
            let path: Option<PathBuf> = Some(file.path().to_path_buf());
            #[cfg(target_arch = "wasm32")]
            let path: Option<PathBuf> = None;
            let data = file.read().await;
            push_event(&queue, IoEvent::MeshLoaded { file_name, data, path });
            ctx.request_repaint();
        }
    });
}

pub fn request_open_path(path: PathBuf, queue: IoQueue, ctx: egui::Context) {
    spawn_dialog_task(async move {
        let result = std::fs::read(&path)
            .map(|data| {
                let file_name = path
                    .file_name()
                    .unwrap_or_default()
                    .to_string_lossy()
                    .to_string();
                IoEvent::MeshLoaded { file_name, data, path: Some(path.clone()) }
            })
            .unwrap_or_else(|error| IoEvent::Error(format!("Failed to load mesh: {}", error)));
        push_event(&queue, result);
        ctx.request_repaint();
    });
}

pub fn request_save_dialog(mesh: Mesh, file_name: String, format: MshSaveFormat) {
    spawn_dialog_task(async move {
        let mut data = Vec::new();
        let write_result = match format {
            MshSaveFormat::V2 => rmsh_io::write_msh_v2(&mut data, &mesh),
            MshSaveFormat::V4 => rmsh_io::write_msh_v4(&mut data, &mesh),
        };

        if let Err(error) = write_result {
            log::error!("Failed to serialize mesh: {}", error);
            return;
        }

        let file_handle = rfd::AsyncFileDialog::new()
            .add_filter("Gmsh Mesh", &["msh"])
            .set_file_name(&file_name)
            .set_title("Save Mesh File")
            .save_file()
            .await;

        if let Some(file_handle) = file_handle {
            if let Err(error) = file_handle.write(&data).await {
                log::error!("Failed to save mesh: {}", error);
            } else {
                log::info!("Saved mesh: {}", file_name);
            }
        }
    });
}

pub fn default_save_name(mesh_name: Option<&str>, format: MshSaveFormat) -> String {
    let stem = mesh_name
        .and_then(|name| std::path::Path::new(name).file_stem())
        .and_then(|stem| stem.to_str())
        .filter(|stem| !stem.is_empty())
        .unwrap_or("mesh");
    match format {
        MshSaveFormat::V2 => format!("{}_v2.msh", stem),
        MshSaveFormat::V4 => format!("{}_v4.msh", stem),
    }
}

fn push_event(queue: &IoQueue, event: IoEvent) {
    if let Ok(mut queue) = queue.lock() {
        queue.push_back(event);
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn spawn_dialog_task<F>(future: F)
where
    F: std::future::Future<Output = ()> + Send + 'static,
{
    std::thread::spawn(move || {
        pollster::block_on(future);
    });
}

#[cfg(target_arch = "wasm32")]
fn spawn_dialog_task<F>(future: F)
where
    F: std::future::Future<Output = ()> + 'static,
{
    wasm_bindgen_futures::spawn_local(future);
}