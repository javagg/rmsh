/// Orbit camera for 3D scene navigation.
pub struct OrbitCamera {
    /// Target point the camera orbits around.
    pub target: nalgebra::Point3<f32>,
    /// Distance from target.
    pub distance: f32,
    /// Yaw angle in radians.
    pub yaw: f32,
    /// Pitch angle in radians (clamped to avoid gimbal lock).
    pub pitch: f32,
    /// Field of view in radians.
    pub fov: f32,
    /// Near clipping plane.
    pub near: f32,
    /// Far clipping plane.
    pub far: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            target: nalgebra::Point3::origin(),
            distance: 5.0,
            yaw: std::f32::consts::FRAC_PI_4,
            pitch: std::f32::consts::FRAC_PI_6,
            fov: std::f32::consts::FRAC_PI_4,
            near: 0.01,
            far: 1000.0,
        }
    }
}

impl OrbitCamera {
    pub fn new() -> Self {
        Self::default()
    }

    /// Rotate the camera by delta yaw/pitch (in radians).
    pub fn rotate(&mut self, delta_yaw: f32, delta_pitch: f32) {
        self.yaw += delta_yaw;
        self.pitch = (self.pitch + delta_pitch).clamp(
            -std::f32::consts::FRAC_PI_2 + 0.01,
            std::f32::consts::FRAC_PI_2 - 0.01,
        );
    }

    /// Zoom in/out by a factor (positive = zoom in).
    pub fn zoom(&mut self, delta: f32) {
        self.distance = (self.distance * (1.0 - delta * 0.1)).max(0.1);
    }

    /// Pan the camera target in the screen plane.
    pub fn pan(&mut self, delta_x: f32, delta_y: f32) {
        let right = self.right_vector();
        let up = self.up_vector();
        let scale = self.distance * 0.002;
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

    /// Compute projection matrix.
    pub fn projection_matrix(&self, aspect: f32) -> nalgebra::Matrix4<f32> {
        nalgebra::Matrix4::new_perspective(aspect, self.fov, self.near, self.far)
    }

    /// Compute combined view-projection matrix.
    pub fn view_projection_matrix(&self, aspect: f32) -> nalgebra::Matrix4<f32> {
        self.projection_matrix(aspect) * self.view_matrix()
    }

    /// Fit the camera to view the given bounding box.
    pub fn fit_to_bbox(&mut self, center: [f32; 3], diagonal: f32) {
        self.target = nalgebra::Point3::new(center[0], center[1], center[2]);
        self.distance = diagonal * 1.5;
        self.near = diagonal * 0.001;
        self.far = diagonal * 100.0;
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
