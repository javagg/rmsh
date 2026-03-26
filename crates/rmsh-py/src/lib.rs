use pyo3::exceptions::PyNotImplementedError;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyTuple};

macro_rules! stub_pyfunction {
    ($rust_name:ident, $py_name:literal, $prototype:literal) => {
        #[pyfunction]
        #[pyo3(name = $py_name, signature = (*args, **kwargs))]
        fn $rust_name(
            args: &pyo3::Bound<'_, PyTuple>,
            kwargs: Option<&pyo3::Bound<'_, PyDict>>,
        ) -> pyo3::PyResult<()> {
            let _ = (args, kwargs);
            Err(PyNotImplementedError::new_err(format!(
                "{} is not implemented yet",
                $prototype
            )))
        }
    };
}

stub_pyfunction!(initialize_impl, "initialize", "rmshInitialize(int argc, const char *const argv[], int readConfigFiles, int run, int *ierr)");
stub_pyfunction!(finalize_impl, "finalize", "rmshFinalize(int *ierr)");
stub_pyfunction!(clear_impl, "clear", "rmshClear(int *ierr)");
stub_pyfunction!(open_impl, "open", "rmshOpen(const char *fileName, int *ierr)");
stub_pyfunction!(merge_impl, "merge", "rmshMerge(const char *fileName, int *ierr)");
stub_pyfunction!(write_impl, "write", "rmshWrite(const char *fileName, int *ierr)");

stub_pyfunction!(option_set_number_impl, "option_set_number", "rmshOptionSetNumber(const char *name, double value, int *ierr)");
stub_pyfunction!(option_get_number_impl, "option_get_number", "rmshOptionGetNumber(const char *name, double *value, int *ierr)");
stub_pyfunction!(option_set_string_impl, "option_set_string", "rmshOptionSetString(const char *name, const char *value, int *ierr)");
stub_pyfunction!(option_get_string_impl, "option_get_string", "rmshOptionGetString(const char *name, char **value, int *ierr)");
stub_pyfunction!(option_set_color_impl, "option_set_color", "rmshOptionSetColor(const char *name, int r, int g, int b, int a, int *ierr)");
stub_pyfunction!(option_get_color_impl, "option_get_color", "rmshOptionGetColor(const char *name, int *r, int *g, int *b, int *a, int *ierr)");

stub_pyfunction!(logger_start_impl, "logger_start", "rmshLoggerStart(int *ierr)");
stub_pyfunction!(logger_stop_impl, "logger_stop", "rmshLoggerStop(int *ierr)");
stub_pyfunction!(logger_get_impl, "logger_get", "rmshLoggerGet(char ***log, size_t *log_n, int *ierr)");

stub_pyfunction!(model_add_impl, "model_add", "rmshModelAdd(const char *name, int *ierr)");
stub_pyfunction!(model_remove_impl, "model_remove", "rmshModelRemove(int *ierr)");
stub_pyfunction!(model_get_current_impl, "model_get_current", "rmshModelGetCurrent(char **name, int *ierr)");
stub_pyfunction!(model_set_current_impl, "model_set_current", "rmshModelSetCurrent(const char *name, int *ierr)");
stub_pyfunction!(model_get_dimension_impl, "model_get_dimension", "rmshModelGetDimension(int *dim, int *ierr)");
stub_pyfunction!(model_get_entities_impl, "model_get_entities", "rmshModelGetEntities(int **dimTags, size_t *dimTags_n, int dim, int *ierr)");
stub_pyfunction!(model_get_entity_name_impl, "model_get_entity_name", "rmshModelGetEntityName(int dim, int tag, char **name, int *ierr)");
stub_pyfunction!(model_set_entity_name_impl, "model_set_entity_name", "rmshModelSetEntityName(int dim, int tag, const char *name, int *ierr)");
stub_pyfunction!(model_get_bounding_box_impl, "model_get_bounding_box", "rmshModelGetBoundingBox(int dim, int tag, double *xmin, double *ymin, double *zmin, double *xmax, double *ymax, double *zmax, int *ierr)");
stub_pyfunction!(model_add_physical_group_impl, "model_add_physical_group", "rmshModelAddPhysicalGroup(int dim, const int *tags, size_t tags_n, int tag, const char *name, int *ierr)");
stub_pyfunction!(model_get_physical_groups_impl, "model_get_physical_groups", "rmshModelGetPhysicalGroups(int **dimTags, size_t *dimTags_n, int dim, int *ierr)");
stub_pyfunction!(model_set_physical_name_impl, "model_set_physical_name", "rmshModelSetPhysicalName(int dim, int tag, const char *name, int *ierr)");
stub_pyfunction!(model_get_physical_name_impl, "model_get_physical_name", "rmshModelGetPhysicalName(int dim, int tag, char **name, int *ierr)");

stub_pyfunction!(model_geo_add_point_impl, "model_geo_add_point", "rmshModelGeoAddPoint(double x, double y, double z, double meshSize, int tag, int *ierr)");
stub_pyfunction!(model_geo_add_line_impl, "model_geo_add_line", "rmshModelGeoAddLine(int startTag, int endTag, int tag, int *ierr)");
stub_pyfunction!(model_geo_add_curve_loop_impl, "model_geo_add_curve_loop", "rmshModelGeoAddCurveLoop(const int *curveTags, size_t curveTags_n, int tag, int *ierr)");
stub_pyfunction!(model_geo_add_plane_surface_impl, "model_geo_add_plane_surface", "rmshModelGeoAddPlaneSurface(const int *wireTags, size_t wireTags_n, int tag, int *ierr)");
stub_pyfunction!(model_geo_synchronize_impl, "model_geo_synchronize", "rmshModelGeoSynchronize(int *ierr)");

stub_pyfunction!(model_occ_add_box_impl, "model_occ_add_box", "rmshModelOccAddBox(double x, double y, double z, double dx, double dy, double dz, int tag, int *ierr)");
stub_pyfunction!(model_occ_add_sphere_impl, "model_occ_add_sphere", "rmshModelOccAddSphere(double x, double y, double z, double r, int tag, int *ierr)");
stub_pyfunction!(model_occ_add_cylinder_impl, "model_occ_add_cylinder", "rmshModelOccAddCylinder(double x, double y, double z, double dx, double dy, double dz, double r, int tag, int *ierr)");
stub_pyfunction!(model_occ_cut_impl, "model_occ_cut", "rmshModelOccCut(const int *objectDimTags, size_t objectDimTags_n, const int *toolDimTags, size_t toolDimTags_n, int *ierr)");
stub_pyfunction!(model_occ_fuse_impl, "model_occ_fuse", "rmshModelOccFuse(const int *objectDimTags, size_t objectDimTags_n, const int *toolDimTags, size_t toolDimTags_n, int *ierr)");
stub_pyfunction!(model_occ_fragment_impl, "model_occ_fragment", "rmshModelOccFragment(const int *objectDimTags, size_t objectDimTags_n, const int *toolDimTags, size_t toolDimTags_n, int *ierr)");
stub_pyfunction!(model_occ_synchronize_impl, "model_occ_synchronize", "rmshModelOccSynchronize(int *ierr)");

stub_pyfunction!(model_mesh_set_size_impl, "model_mesh_set_size", "rmshModelMeshSetSize(const int *dimTags, size_t dimTags_n, double size, int *ierr)");
stub_pyfunction!(model_mesh_generate_impl, "model_mesh_generate", "rmshModelMeshGenerate(int dim, int *ierr)");
stub_pyfunction!(model_mesh_set_order_impl, "model_mesh_set_order", "rmshModelMeshSetOrder(int order, int *ierr)");
stub_pyfunction!(model_mesh_get_nodes_impl, "model_mesh_get_nodes", "rmshModelMeshGetNodes(size_t *nodeTags_n, size_t *coord_n, size_t *parametricCoord_n, int dim, int tag, int includeBoundary, int returnParametricCoord, int *ierr)");
stub_pyfunction!(model_mesh_get_elements_impl, "model_mesh_get_elements", "rmshModelMeshGetElements(size_t *elementTypes_n, size_t *elementTags_n, size_t *nodeTags_n, int dim, int tag, int *ierr)");
stub_pyfunction!(model_mesh_clear_impl, "model_mesh_clear", "rmshModelMeshClear(const int *dimTags, size_t dimTags_n, int *ierr)");
stub_pyfunction!(model_mesh_optimize_impl, "model_mesh_optimize", "rmshModelMeshOptimize(const char *method, int force, int niter, const int *dimTags, size_t dimTags_n, int *ierr)");
stub_pyfunction!(model_mesh_refine_impl, "model_mesh_refine", "rmshModelMeshRefine(int *ierr)");
stub_pyfunction!(model_mesh_recombine_impl, "model_mesh_recombine", "rmshModelMeshRecombine(int dim, int tag, double angle, int *ierr)");

stub_pyfunction!(plugin_set_number_impl, "plugin_set_number", "rmshPluginSetNumber(const char *name, const char *option, double value, int *ierr)");
stub_pyfunction!(plugin_set_string_impl, "plugin_set_string", "rmshPluginSetString(const char *name, const char *option, const char *value, int *ierr)");
stub_pyfunction!(plugin_run_impl, "plugin_run", "rmshPluginRun(const char *name, int *ierr)");

stub_pyfunction!(gui_initialize_impl, "gui_initialize", "rmshGuiInitialize(int *ierr)");
stub_pyfunction!(gui_run_impl, "gui_run", "rmshGuiRun(int *ierr)");
stub_pyfunction!(gui_wait_impl, "gui_wait", "rmshGuiWait(double time, int *ierr)");

#[pyo3::pymodule]
fn _rmsh(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(pyo3::wrap_pyfunction!(initialize_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(finalize_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(clear_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(open_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(merge_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(write_impl, m)?)?;

    m.add_function(pyo3::wrap_pyfunction!(option_set_number_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(option_get_number_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(option_set_string_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(option_get_string_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(option_set_color_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(option_get_color_impl, m)?)?;

    m.add_function(pyo3::wrap_pyfunction!(logger_start_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(logger_stop_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(logger_get_impl, m)?)?;

    m.add_function(pyo3::wrap_pyfunction!(model_add_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_remove_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_get_current_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_set_current_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_get_dimension_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_get_entities_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_get_entity_name_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_set_entity_name_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_get_bounding_box_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_add_physical_group_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_get_physical_groups_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_set_physical_name_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_get_physical_name_impl, m)?)?;

    m.add_function(pyo3::wrap_pyfunction!(model_geo_add_point_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_geo_add_line_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_geo_add_curve_loop_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_geo_add_plane_surface_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_geo_synchronize_impl, m)?)?;

    m.add_function(pyo3::wrap_pyfunction!(model_occ_add_box_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_occ_add_sphere_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_occ_add_cylinder_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_occ_cut_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_occ_fuse_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_occ_fragment_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_occ_synchronize_impl, m)?)?;

    m.add_function(pyo3::wrap_pyfunction!(model_mesh_set_size_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_mesh_generate_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_mesh_set_order_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_mesh_get_nodes_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_mesh_get_elements_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_mesh_clear_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_mesh_optimize_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_mesh_refine_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(model_mesh_recombine_impl, m)?)?;

    m.add_function(pyo3::wrap_pyfunction!(plugin_set_number_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(plugin_set_string_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(plugin_run_impl, m)?)?;

    m.add_function(pyo3::wrap_pyfunction!(gui_initialize_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(gui_run_impl, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(gui_wait_impl, m)?)?;

    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}
