use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::vertex::{EdgeType, Graph, VertexId};

#[pyclass(name = "EdgeType", eq, eq_int, frozen, hash)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PyEdgeType {
    Undirected = 0,
    Directed = 1,
}

impl From<PyEdgeType> for EdgeType {
    fn from(value: PyEdgeType) -> Self {
        match value {
            PyEdgeType::Undirected => EdgeType::Undirected,
            PyEdgeType::Directed => EdgeType::Directed,
        }
    }
}

#[pyclass(name = "Graph")]
pub struct PyGraph {
    inner: Graph<(), ()>,
}

#[pymethods]
impl PyGraph {
    #[new]
    fn new() -> Self {
        Self {
            inner: Graph::new(),
        }
    }

    fn add_vertex(&mut self) -> u32 {
        self.inner.add_vertex(()).value()
    }

    fn vertex_count(&self) -> usize {
        self.inner.vertex_count()
    }

    fn edge_count(&self) -> usize {
        self.inner.edge_count()
    }

    fn has_vertex(&self, vertex: u32) -> bool {
        self.inner.has_vertex(VertexId::new(vertex))
    }

    #[pyo3(signature = (source, target, weight=1.0, edge_type=PyEdgeType::Undirected))]
    fn add_edge(
        &mut self,
        source: u32,
        target: u32,
        weight: f64,
        edge_type: PyEdgeType,
    ) -> PyResult<usize> {
        let source = VertexId::new(source);
        let target = VertexId::new(target);
        if !self.inner.has_vertex(source) {
            return Err(PyValueError::new_err("source vertex does not exist"));
        }
        if !self.inner.has_vertex(target) {
            return Err(PyValueError::new_err("target vertex does not exist"));
        }
        Ok(self
            .inner
            .add_edge(source, target, weight, edge_type.into(), ()))
    }

    fn has_edge(&self, source: u32, target: u32) -> bool {
        self.inner
            .has_edge(VertexId::new(source), VertexId::new(target))
    }

    fn degree(&self, vertex: u32) -> PyResult<usize> {
        let vertex = VertexId::new(vertex);
        if !self.inner.has_vertex(vertex) {
            return Err(PyValueError::new_err("vertex does not exist"));
        }
        Ok(self.inner.degree(vertex))
    }

    fn clear(&mut self) {
        self.inner.clear();
    }

    fn __len__(&self) -> usize {
        self.inner.vertex_count()
    }
}

pub(super) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyEdgeType>()?;
    m.add_class::<PyGraph>()?;
    Ok(())
}
