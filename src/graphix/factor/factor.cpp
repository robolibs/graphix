#include "graphix/factor/factor.hpp"

namespace graphix {
    namespace factor {

        Factor::Factor() = default;

        Factor::Factor(const std::vector<Key> &keys) : m_keys(keys) {}

        const std::vector<Key> &Factor::keys() const { return m_keys; }

        size_t Factor::key_count() const { return m_keys.size(); }

    } // namespace factor
} // namespace graphix
