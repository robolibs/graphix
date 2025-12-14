#include "graphix/factor/factor.hpp"
#include "graphix/factor/graph.hpp"
#include "graphix/vertice/graph.hpp"
#include <doctest/doctest.h>

TEST_CASE("Vertice Graph exists") {
    graphix::vertice::Graph vgraph;
    CHECK(vgraph.node_count() == 0);
    CHECK(vgraph.edge_count() == 0);
}

TEST_CASE("Factor Graph exists") {
    graphix::factor::Graph fgraph;
    CHECK(fgraph.factor_count() == 0);
}

TEST_CASE("Factor exists") {
    graphix::factor::Factor factor;
    CHECK(factor.key_count() == 0);
}
