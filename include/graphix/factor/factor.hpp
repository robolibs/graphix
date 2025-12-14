#pragma once

#include "graphix/kernel.hpp"
#include "graphix/smallvec.hpp"
#include <initializer_list>

namespace graphix {
    namespace factor {

        // Default: most factors connect 1-6 variables
        constexpr size_t DEFAULT_FACTOR_SIZE = 6;

        class Factor {
          public:
            Factor();
            explicit Factor(std::initializer_list<Key> keys);

            const SmallVec<Key, DEFAULT_FACTOR_SIZE> &keys() const;
            size_t key_count() const;

            void add_key(Key key);

          protected:
            SmallVec<Key, DEFAULT_FACTOR_SIZE> m_keys;
        };

    } // namespace factor
} // namespace graphix
