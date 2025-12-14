#include "graphix/factor/graph.hpp"
#include "graphix/vertex/graph.hpp"
#include <iostream>

int main() {
    graphix::vertex::Graph vertex_graph;
    graphix::factor::Graph factor_graph;

    std::cout << "Graphix base structure is here!" << std::endl;
    std::cout << "  - graphix::vertex::Graph" << std::endl;
    std::cout << "  - graphix::factor::Graph" << std::endl;

    return 0;
}
