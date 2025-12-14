#include "graphix/factor/graph.hpp"

namespace graphix {
    namespace factor {

        Graph::Graph() = default;

        void Graph::add_factor(std::shared_ptr<Factor> factor) { m_factors.push_back(factor); }

        size_t Graph::factor_count() const { return m_factors.size(); }

    } // namespace factor
} // namespace graphix
