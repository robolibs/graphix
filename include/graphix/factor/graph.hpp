#pragma once

#include "graphix/factor/factor.hpp"
#include "graphix/kernel.hpp"
#include <memory>
#include <set>
#include <vector>

namespace graphix {
    namespace factor {

        // Templated FactorGraph for type safety
        // Can be Graph<Factor>, Graph<NonlinearFactor>, etc.
        template <typename FACTOR = Factor> class Graph {
          public:
            using FactorPtr = std::shared_ptr<FACTOR>;

            Graph() = default;

            // Add a factor to the graph
            void add(FactorPtr factor) {
                if (factor) {
                    m_factors.push_back(factor);
                }
            }

            // Number of factors
            size_t size() const { return m_factors.size(); }

            bool empty() const { return m_factors.empty(); }

            // Access factor by index
            const FactorPtr &operator[](size_t index) const { return m_factors[index]; }

            FactorPtr &operator[](size_t index) { return m_factors[index]; }

            const FactorPtr &at(size_t index) const { return m_factors.at(index); }

            // Get all unique keys (variables) in the graph
            std::set<Key> keys() const {
                std::set<Key> all_keys;
                for (const auto &factor : m_factors) {
                    if (factor) {
                        for (size_t i = 0; i < factor->size(); ++i) {
                            all_keys.insert(factor->keys()[i]);
                        }
                    }
                }
                return all_keys;
            }

            // Iterator support
            using const_iterator = typename std::vector<FactorPtr>::const_iterator;
            using iterator = typename std::vector<FactorPtr>::iterator;

            const_iterator begin() const { return m_factors.begin(); }
            const_iterator end() const { return m_factors.end(); }

            iterator begin() { return m_factors.begin(); }
            iterator end() { return m_factors.end(); }

            // Clear all factors
            void clear() { m_factors.clear(); }

          private:
            std::vector<FactorPtr> m_factors;
        };

    } // namespace factor
} // namespace graphix
