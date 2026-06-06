//! Python bindings for graphix (pyo3).

mod graph;

use pyo3::prelude::*;
use pyo3::types::PyModule;

pub fn register_python_module(m: &Bound<'_, PyModule>) -> PyResult<()> {
    graph::register(m)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}

#[pymodule]
fn graphix(m: &Bound<'_, PyModule>) -> PyResult<()> {
    register_python_module(m)
}
