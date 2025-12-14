#include "graphix/factor/factor.hpp"
#include <algorithm>

namespace graphix {
    namespace factor {

        Factor::Factor() = default;

        Factor::Factor(std::initializer_list<Key> keys) : m_keys(keys) {}

        const SmallVec<Key, DEFAULT_FACTOR_SIZE> &Factor::keys() const { return m_keys; }

        size_t Factor::size() const { return m_keys.size(); }

        bool Factor::involves(Key key) const { return std::find(m_keys.begin(), m_keys.end(), key) != m_keys.end(); }

    } // namespace factor
} // namespace graphix
