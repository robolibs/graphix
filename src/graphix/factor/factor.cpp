#include "graphix/factor/factor.hpp"

namespace graphix {
    namespace factor {

        Factor::Factor() = default;

        Factor::Factor(std::initializer_list<Key> keys) : m_keys(keys) {}

        const SmallVec<Key, DEFAULT_FACTOR_SIZE> &Factor::keys() const { return m_keys; }

        size_t Factor::key_count() const { return m_keys.size(); }

        void Factor::add_key(Key key) { m_keys.push_back(key); }

    } // namespace factor
} // namespace graphix
