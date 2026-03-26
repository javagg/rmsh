use pyo3::prelude::*;

include!(concat!(env!("OUT_DIR"), "/generated_bindings.rs"));

#[pymodule]
fn _rmsh(m: &Bound<'_, PyModule>) -> PyResult<()> {
    register_generated(m)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}
