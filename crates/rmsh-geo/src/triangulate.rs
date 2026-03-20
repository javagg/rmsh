/// Triangulate a polygon face given as a list of vertex positions.
/// Uses simple fan triangulation from the first vertex.
/// Returns triangle indices relative to the input slice.
pub fn fan_triangulate(vertex_count: usize) -> Vec<[usize; 3]> {
    if vertex_count < 3 {
        return Vec::new();
    }
    (1..vertex_count - 1)
        .map(|i| [0, i, i + 1])
        .collect()
}
