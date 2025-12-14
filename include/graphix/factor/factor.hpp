#pragma once

#include "graphix/kernel.hpp"
#include <vector>

namespace graphix {
    namespace factor {

        class Factor {
          public:
            Factor();
            explicit Factor(const std::vector<Key> &keys);

            const std::vector<Key> &keys() const;
            size_t key_count() const;

          protected:
            std::vector<Key> m_keys;
        };

    } // namespace factor
} // namespace graphix
