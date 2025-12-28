#pragma once

#include "graphix/kernel.hpp"
#include "graphix/smallvec.hpp"
#include <algorithm>
#include <initializer_list>
#include <vector>

namespace graphix {
    namespace factor {

        constexpr size_t DEFAULT_FACTOR_SIZE = 6;

        // Factor base class: pure structure (which variables are connected)
        // NO error computation here - that's in NonlinearFactor
        class Factor {
          public:
            inline Factor() = default;
            inline explicit Factor(std::initializer_list<Key> keys) : m_keys(keys) {}

            // Variadic constructor for convenience
            template <typename... Keys> explicit Factor(Keys... keys) : m_keys{static_cast<Key>(keys)...} {}

            virtual ~Factor() = default;

            // Access keys
            inline const SmallVec<Key, DEFAULT_FACTOR_SIZE> &keys() const { return m_keys; }
            inline size_t size() const { return m_keys.size(); }

            // Check if factor involves a specific key
            inline bool involves(Key key) const { return std::find(m_keys.begin(), m_keys.end(), key) != m_keys.end(); }

          protected:
            SmallVec<Key, DEFAULT_FACTOR_SIZE> m_keys;
        };

    } // namespace factor
} // namespace graphix
