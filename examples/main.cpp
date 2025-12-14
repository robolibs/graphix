#include "graphix/factor/graph.hpp"
#include "graphix/vertice/graph.hpp"
#include <iostream>

int main() {
    graphix::vertice::Graph vertice_graph;
    graphix::factor::Graph factor_graph;

    std::cout << "Graphix base structure is here!" << std::endl;
    std::cout << "  - graphix::vertice::Graph" << std::endl;
    std::cout << "  - graphix::factor::Graph" << std::endl;

    return 0;
}
