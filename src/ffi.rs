//! C ABI for graphix.
//!
//! Conventions: opaque Box-backed handles (free with the matching
//! `*_free`); fallible calls return bool/int with the reason in the
//! thread-local `graphix_last_error_message()`.
//!
//! `include/graphix.h` is generated from this file by cbindgen.

// extern "C" fns take raw pointers from C and deref them by design.
#![allow(clippy::not_unsafe_ptr_arg_deref)]

use std::cell::RefCell;
use std::ffi::{CString, c_char};
use std::ptr;

use crate::vertex::{EdgeType, Graph, VertexId};

thread_local! {
    static LAST_ERROR: RefCell<Option<CString>> = const { RefCell::new(None) };
}

fn clear_last_error() {
    LAST_ERROR.with(|slot| *slot.borrow_mut() = None);
}

fn set_last_error(message: impl Into<String>) {
    let message = message.into().replace('\0', " ");
    LAST_ERROR.with(|slot| {
        *slot.borrow_mut() = Some(
            CString::new(message).unwrap_or_else(|_| CString::new("graphix ffi error").unwrap()),
        );
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn graphix_last_error_message() -> *const c_char {
    LAST_ERROR.with(|slot| {
        slot.borrow()
            .as_ref()
            .map(|m| m.as_ptr())
            .unwrap_or(ptr::null())
    })
}

/// Opaque graph handle for unit-property vertex graphs.
pub struct GraphixGraph {
    inner: Graph<(), ()>,
}

/// FFI-safe vertex id.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GraphixVertex {
    pub value: u32,
}

impl From<GraphixVertex> for VertexId<()> {
    fn from(value: GraphixVertex) -> Self {
        VertexId::new(value.value)
    }
}

/// FFI edge direction selector.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GraphixEdgeType {
    Undirected = 0,
    Directed = 1,
}

impl From<GraphixEdgeType> for EdgeType {
    fn from(value: GraphixEdgeType) -> Self {
        match value {
            GraphixEdgeType::Undirected => EdgeType::Undirected,
            GraphixEdgeType::Directed => EdgeType::Directed,
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn graphix_graph_new() -> *mut GraphixGraph {
    clear_last_error();
    Box::into_raw(Box::new(GraphixGraph {
        inner: Graph::new(),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn graphix_graph_free(graph: *mut GraphixGraph) {
    if graph.is_null() {
        return;
    }
    // SAFETY: pointer came from graphix_graph_new.
    unsafe { drop(Box::from_raw(graph)) };
}

#[unsafe(no_mangle)]
pub extern "C" fn graphix_graph_add_vertex(graph: *mut GraphixGraph) -> GraphixVertex {
    clear_last_error();
    let Some(graph) = (unsafe { graph.as_mut() }) else {
        set_last_error("null graph handle");
        return GraphixVertex { value: u32::MAX };
    };
    GraphixVertex {
        value: graph.inner.add_vertex(()).value(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn graphix_graph_vertex_count(graph: *const GraphixGraph) -> usize {
    clear_last_error();
    let Some(graph) = (unsafe { graph.as_ref() }) else {
        set_last_error("null graph handle");
        return 0;
    };
    graph.inner.vertex_count()
}

#[unsafe(no_mangle)]
pub extern "C" fn graphix_graph_edge_count(graph: *const GraphixGraph) -> usize {
    clear_last_error();
    let Some(graph) = (unsafe { graph.as_ref() }) else {
        set_last_error("null graph handle");
        return 0;
    };
    graph.inner.edge_count()
}

#[unsafe(no_mangle)]
pub extern "C" fn graphix_graph_has_vertex(
    graph: *const GraphixGraph,
    vertex: GraphixVertex,
) -> bool {
    clear_last_error();
    let Some(graph) = (unsafe { graph.as_ref() }) else {
        set_last_error("null graph handle");
        return false;
    };
    graph.inner.has_vertex(vertex.into())
}

#[unsafe(no_mangle)]
pub extern "C" fn graphix_graph_add_edge(
    graph: *mut GraphixGraph,
    source: GraphixVertex,
    target: GraphixVertex,
    weight: f64,
    edge_type: GraphixEdgeType,
    out_edge_id: *mut usize,
) -> bool {
    clear_last_error();
    let Some(graph) = (unsafe { graph.as_mut() }) else {
        set_last_error("null graph handle");
        return false;
    };
    let source_id = VertexId::new(source.value);
    let target_id = VertexId::new(target.value);
    if !graph.inner.has_vertex(source_id) {
        set_last_error("source vertex does not exist");
        return false;
    }
    if !graph.inner.has_vertex(target_id) {
        set_last_error("target vertex does not exist");
        return false;
    }
    let edge = graph
        .inner
        .add_edge(source_id, target_id, weight, edge_type.into(), ());
    if !out_edge_id.is_null() {
        // SAFETY: caller supplied writable storage or NULL.
        unsafe { *out_edge_id = edge };
    }
    true
}

#[unsafe(no_mangle)]
pub extern "C" fn graphix_graph_has_edge(
    graph: *const GraphixGraph,
    source: GraphixVertex,
    target: GraphixVertex,
) -> bool {
    clear_last_error();
    let Some(graph) = (unsafe { graph.as_ref() }) else {
        set_last_error("null graph handle");
        return false;
    };
    graph.inner.has_edge(source.into(), target.into())
}

#[unsafe(no_mangle)]
pub extern "C" fn graphix_graph_degree(graph: *const GraphixGraph, vertex: GraphixVertex) -> usize {
    clear_last_error();
    let Some(graph) = (unsafe { graph.as_ref() }) else {
        set_last_error("null graph handle");
        return 0;
    };
    if !graph.inner.has_vertex(vertex.into()) {
        set_last_error("vertex does not exist");
        return 0;
    }
    graph.inner.degree(vertex.into())
}

#[unsafe(no_mangle)]
pub extern "C" fn graphix_graph_clear(graph: *mut GraphixGraph) -> bool {
    clear_last_error();
    let Some(graph) = (unsafe { graph.as_mut() }) else {
        set_last_error("null graph handle");
        return false;
    };
    graph.inner.clear();
    true
}
