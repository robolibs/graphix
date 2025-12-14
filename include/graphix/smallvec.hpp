#pragma once

#include <array>
#include <cstddef>
#include <initializer_list>
#include <vector>

namespace graphix {

    // ============================================================================
    // SmallVec<T, N>: Small vector optimization (like LLVM SmallVector)
    // Stores up to N elements inline, falls back to heap for larger sizes
    // ============================================================================

    template <typename T, size_t N> class SmallVec {
      public:
        SmallVec() = default;

        SmallVec(std::initializer_list<T> init) {
            for (const auto &val : init) {
                push_back(val);
            }
        }

        void push_back(const T &value) {
            if (m_using_heap) {
                m_heap.push_back(value);
                m_size = m_heap.size();
            } else {
                if (m_size < N) {
                    m_inline[m_size++] = value;
                } else {
                    // Transition to heap
                    m_heap.reserve(m_size + 1);
                    m_heap.assign(m_inline.begin(), m_inline.begin() + m_size);
                    m_heap.push_back(value);
                    m_using_heap = true;
                    m_size = m_heap.size();
                }
            }
        }

        T &operator[](size_t i) { return m_using_heap ? m_heap[i] : m_inline[i]; }

        const T &operator[](size_t i) const { return m_using_heap ? m_heap[i] : m_inline[i]; }

        size_t size() const { return m_size; }

        bool empty() const { return m_size == 0; }

        void clear() {
            m_size = 0;
            m_using_heap = false;
            m_heap.clear();
        }

        // Iterator support
        T *begin() { return m_using_heap ? m_heap.data() : m_inline.data(); }

        const T *begin() const { return m_using_heap ? m_heap.data() : m_inline.data(); }

        T *end() { return begin() + m_size; }

        const T *end() const { return begin() + m_size; }

      private:
        size_t m_size = 0;
        bool m_using_heap = false;
        std::array<T, N> m_inline{};
        std::vector<T> m_heap;
    };

} // namespace graphix
