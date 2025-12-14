#pragma once

#include "graphix/kernel.hpp"
#include "graphix/smallvec.hpp"
#include <initializer_list>
#include <vector>

namespace graphix {
    namespace factor {

        constexpr size_t DEFAULT_FACTOR_SIZE = 6;

        // Factor base class: pure structure (which variables are connected)
        // NO error computation here - that's in NonlinearFactor
        class Factor {
          public:
            Factor();
            explicit Factor(std::initializer_list<Key> keys);

            // Variadic constructor for convenience
            template <typename... Keys> explicit Factor(Keys... keys) : m_keys{static_cast<Key>(keys)...} {}

            virtual ~Factor() = default;

            // Access keys
            const SmallVec<Key, DEFAULT_FACTOR_SIZE> &keys() const;
            size_t size() const;

            // Check if factor involves a specific key
            bool involves(Key key) const;

          protected:
            SmallVec<Key, DEFAULT_FACTOR_SIZE> m_keys;
        };

    } // namespace factor
} // namespace graphix
