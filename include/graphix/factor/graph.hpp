#pragma once

#include "graphix/kernel.hpp"
#include <memory>
#include <vector>

namespace graphix {
    namespace factor {

        class Factor; // Forward declaration

        class Graph {
          public:
            Graph();

            void add_factor(std::shared_ptr<Factor> factor);
            size_t factor_count() const;

          private:
            std::vector<std::shared_ptr<Factor>> m_factors;
        };

    } // namespace factor
} // namespace graphix
