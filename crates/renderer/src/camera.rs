/// Orbit camera for 3D scene navigation.
pub struct OrbitCamera {
    /// Target point the camera orbits around.
    pub target: nalgebra::Point3<f32>,
    /// Distance from target (used for view matrix and perspective zoom).
    pub distance: f32,
    /// Yaw angle in radians.
    pub yaw: f32,
    /// Pitch angle in radians (clamped to avoid gimbal lock).
    pub pitch: f32,
    /// Field of view in radians (perspective only).
    pub fov: f32,
    /// Near clipping plane.
    pub near: f32,
    /// Far clipping plane.
    pub far: f32,
    /// Whether to use orthographic projection instead of perspective.
    pub orthographic: bool,
    /// Half-height of the orthographic view volume in world units.
    pub ortho_scale: f32,
}

/// Isometric yaw: 45°
const ISO_YAW: f32 = std::f32::consts::FRAC_PI_4;
/// Isometric pitch: arctan(1/√2) ≈ 35.264°
const ISO_PITCH: f32 = 0.6154_8246_f32; // atan(1.0 / sqrt(2.0))

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            target: nalgebra::Point3::origin(),
            distance: 5.0,
            yaw: ISO_YAW,
            pitch: ISO_PITCH,
            fov: std::f32::consts::FRAC_PI_4,
            near: 0.01,
            far: 1000.0,
            orthographic: true,
            ortho_scale: 3.0,
        }
    }
}

impl OrbitCamera {
    pub fn new() -> Self {
        Self::default()
    }

    /// Rotate the camera by delta yaw/pitch (in radians).
    pub fn rotate(&mut self, delta_yaw: f32, delta_pitch: f32) {
        self.yaw += -1.0 * delta_yaw;
        self.pitch = (self.pitch + delta_pitch).clamp(
            -std::f32::consts::FRAC_PI_2 + 0.01,
            std::f32::consts::FRAC_PI_2 - 0.01,
        );
    }

    /// Zoom in/out by a factor (positive = zoom in).
    pub fn zoom(&mut self, delta: f32) {
        if self.orthographic {
            self.ortho_scale = (self.ortho_scale * (1.0 - delta * 0.1)).max(0.001);
        } else {
            self.distance = (self.distance * (1.0 - delta * 0.1)).max(0.1);
        }
    }

    /// Pan the camera target in the screen plane.
    pub fn pan(&mut self, delta_x: f32, delta_y: f32) {
        let right = self.right_vector();
        let up = self.up_vector();
        let scale = if self.orthographic {
            self.ortho_scale * 0.004
        } else {
            self.distance * 0.002
        };
        self.target += right * (-delta_x * scale) + up * (delta_y * scale);
    }

    /// Compute camera eye position.
    pub fn eye_position(&self) -> nalgebra::Point3<f32> {
        let x = self.distance * self.pitch.cos() * self.yaw.sin();
        let y = self.distance * self.pitch.sin();
        let z = self.distance * self.pitch.cos() * self.yaw.cos();
        self.target + nalgebra::Vector3::new(x, y, z)
    }

    /// Compute view matrix.
    pub fn view_matrix(&self) -> nalgebra::Matrix4<f32> {
        let eye = self.eye_position();
        nalgebra::Matrix4::look_at_rh(&eye, &self.target, &nalgebra::Vector3::y())
    }

    /// Compute projection matrix (orthographic or perspective depending on mode).
    pub fn projection_matrix(&self, aspect: f32) -> nalgebra::Matrix4<f32> {
        if self.orthographic {
            let h = self.ortho_scale;
            let w = h * aspect;
            // Use a symmetric near/far centred on the camera to avoid clipping
            // when rotating around an orbit target.
            let depth = self.far;
            nalgebra::Matrix4::new_orthographic(-w, w, -h, h, -depth, depth)
        } else {
            nalgebra::Matrix4::new_perspective(aspect, self.fov, self.near, self.far)
        }
    }

    /// Compute combined view-projection matrix.
    pub fn view_projection_matrix(&self, aspect: f32) -> nalgebra::Matrix4<f32> {
        self.projection_matrix(aspect) * self.view_matrix()
    }

    /// Fit the camera to view the given bounding box.
    /// Preserves the current yaw/pitch and projection mode.
    pub fn fit_to_bbox(&mut self, center: [f32; 3], diagonal: f32) {
        self.target = nalgebra::Point3::new(center[0], center[1], center[2]);
        self.distance = diagonal * 1.5;
        self.near = diagonal * 0.001;
        self.far = diagonal * 100.0;
        // Set ortho scale so the object fills ~2/3 of the view height.
        self.ortho_scale = diagonal * 0.6;
    }

    /// Reset to standard isometric view (45° yaw, 35.264° pitch).
    pub fn set_isometric(&mut self) {
        self.yaw = ISO_YAW;
        self.pitch = ISO_PITCH;
    }

    /// Toggle between orthographic and perspective projection.
    pub fn toggle_projection(&mut self) {
        self.orthographic = !self.orthographic;
        // When switching to perspective, sync distance so the view looks similar.
        if !self.orthographic {
            // distance so that ortho_scale ≈ distance * tan(fov/2)
            self.distance = self.ortho_scale / (self.fov * 0.5).tan();
        }
    }

    fn right_vector(&self) -> nalgebra::Vector3<f32> {
        let view = self.view_matrix();
        nalgebra::Vector3::new(view[(0, 0)], view[(0, 1)], view[(0, 2)])
    }

    fn up_vector(&self) -> nalgebra::Vector3<f32> {
        let view = self.view_matrix();
        nalgebra::Vector3::new(view[(1, 0)], view[(1, 1)], view[(1, 2)])
    }
}
