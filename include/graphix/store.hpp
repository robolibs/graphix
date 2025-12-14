#pragma once

#include "graphix/kernel.hpp"
#include <optional>
#include <vector>

namespace graphix {

    // ============================================================================
    // Store<T>: Stable ID storage with free-list reuse
    // ============================================================================

    template <typename T> class Store {
      public:
        Store() = default;

        // Add new element, returns stable ID
        Id<T> add(const T &value) {
            if (m_free_list.empty()) {
                // No free slots, add to end
                Id<T> id(static_cast<typename Id<T>::value_type>(m_data.size()));
                m_data.push_back(value);
                return id;
            } else {
                // Reuse free slot
                auto slot = m_free_list.back();
                m_free_list.pop_back();
                Id<T> id(slot);
                m_data[slot] = value;
                return id;
            }
        }

        // Remove element (adds to free list)
        void remove(Id<T> id) {
            if (id.is_valid() && id.value() < m_data.size()) {
                m_free_list.push_back(id.value());
            }
        }

        // Access element
        T &operator[](Id<T> id) { return m_data[id.value()]; }

        const T &operator[](Id<T> id) const { return m_data[id.value()]; }

        // Check if ID is valid (not in free list)
        bool contains(Id<T> id) const {
            if (!id.is_valid() || id.value() >= m_data.size()) {
                return false;
            }
            for (auto free_slot : m_free_list) {
                if (free_slot == id.value()) {
                    return false;
                }
            }
            return true;
        }

        size_t size() const { return m_data.size() - m_free_list.size(); }

        bool empty() const { return size() == 0; }

      private:
        std::vector<T> m_data;
        std::vector<typename Id<T>::value_type> m_free_list;
    };

} // namespace graphix
