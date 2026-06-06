#ifndef GRAPHIX_H
#define GRAPHIX_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define Id_INVALID UINT32_MAX

#define DEFAULT_FACTOR_SIZE 6

/**
 * FFI edge direction selector.
 */
typedef enum {
  GRAPHIX_EDGE_TYPE_GRAPHIX_EDGE_TYPE_UNDIRECTED = 0,
  GRAPHIX_EDGE_TYPE_GRAPHIX_EDGE_TYPE_DIRECTED = 1,
} GraphixEdgeType;

/**
 * Opaque graph handle for unit-property vertex graphs.
 */
typedef struct GraphixGraph GraphixGraph;

/**
 * FFI-safe vertex id.
 */
typedef struct {
  uint32_t value;
} GraphixVertex;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

const char *graphix_last_error_message(void);

GraphixGraph *graphix_graph_new(void);

void graphix_graph_free(GraphixGraph *graph);

GraphixVertex graphix_graph_add_vertex(GraphixGraph *graph);

uintptr_t graphix_graph_vertex_count(const GraphixGraph *graph);

uintptr_t graphix_graph_edge_count(const GraphixGraph *graph);

bool graphix_graph_has_vertex(const GraphixGraph *graph, GraphixVertex vertex);

bool graphix_graph_add_edge(GraphixGraph *graph,
                            GraphixVertex source,
                            GraphixVertex target,
                            double weight,
                            GraphixEdgeType edge_type,
                            uintptr_t *out_edge_id);

bool graphix_graph_has_edge(const GraphixGraph *graph, GraphixVertex source, GraphixVertex target);

uintptr_t graphix_graph_degree(const GraphixGraph *graph, GraphixVertex vertex);

bool graphix_graph_clear(GraphixGraph *graph);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  /* GRAPHIX_H */
