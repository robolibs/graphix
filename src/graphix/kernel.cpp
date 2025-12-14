#include "graphix/kernel.hpp"

namespace graphix {

    // Symbol implementation
    Symbol::Symbol(unsigned char c, uint64_t j) {
        // GTSAM-style encoding: chr in upper bits, index in lower bits
        m_key = (static_cast<uint64_t>(c) << 56) | (j & 0x00FFFFFFFFFFFFFF);
    }

    unsigned char Symbol::chr() const { return static_cast<unsigned char>(m_key >> 56); }

    uint64_t Symbol::index() const { return m_key & 0x00FFFFFFFFFFFFFF; }

} // namespace graphix
