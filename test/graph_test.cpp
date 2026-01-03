#include "graphix/factor/factor.hpp"
#include "graphix/factor/graph.hpp"
#include "graphix/vertex/graph.hpp"
#include <doctest/doctest.h>

TEST_CASE("Vertex Graph exists") {
    graphix::vertex::Graph<void> vgraph;
    CHECK(vgraph.vertex_count() == 0);
}

TEST_CASE("Factor Graph exists") {
    graphix::factor::Graph<> fgraph;
    CHECK(fgraph.size() == 0);
}

TEST_CASE("Factor exists") {
    graphix::factor::Factor factor;
    CHECK(factor.size() == 0);
}
