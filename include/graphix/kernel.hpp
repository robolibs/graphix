#pragma once

#include <cstdint>
#include <functional>

namespace graphix {

    // ============================================================================
    // Key: Stable identifier for variables/nodes (GTSAM-style)
    // ============================================================================

    using Key = uint64_t;

    // ============================================================================
    // Symbol: Encode character + index into a Key (e.g., X(0), L(12))
    // ============================================================================

    class Symbol {
      public:
        Symbol() : m_key(0) {}
        inline Symbol(unsigned char c, uint64_t j) {
            // GTSAM-style encoding: chr in upper bits, index in lower bits
            m_key = (static_cast<uint64_t>(c) << 56) | (j & 0x00FFFFFFFFFFFFFF);
        }

        inline unsigned char chr() const { return static_cast<unsigned char>(m_key >> 56); }
        inline uint64_t index() const { return m_key & 0x00FFFFFFFFFFFFFF; }
        Key key() const { return m_key; }

        operator Key() const { return m_key; }

        bool operator==(const Symbol &other) const { return m_key == other.m_key; }
        bool operator!=(const Symbol &other) const { return m_key != other.m_key; }

      private:
        Key m_key;
    };

    // Helper functions for symbol creation
    inline Symbol X(uint64_t j) { return Symbol('x', j); }
    inline Symbol L(uint64_t j) { return Symbol('l', j); }
    inline Symbol P(uint64_t j) { return Symbol('p', j); }

    // ============================================================================
    // Id<T>: Strong typed ID wrapper for stable handles
    // ============================================================================

    template <typename T> class Id {
      public:
        using value_type = uint32_t;

        Id() : m_id(invalid_id()) {}
        explicit Id(value_type id) : m_id(id) {}

        value_type value() const { return m_id; }
        bool is_valid() const { return m_id != invalid_id(); }

        bool operator==(const Id &other) const { return m_id == other.m_id; }
        bool operator!=(const Id &other) const { return m_id != other.m_id; }
        bool operator<(const Id &other) const { return m_id < other.m_id; }

        static constexpr value_type invalid_id() { return static_cast<value_type>(-1); }

      private:
        value_type m_id;
    };

} // namespace graphix

// Hash support for Symbol and Id<T>
namespace std {
    template <> struct hash<graphix::Symbol> {
        size_t operator()(const graphix::Symbol &s) const { return hash<graphix::Key>()(s.key()); }
    };

    template <typename T> struct hash<graphix::Id<T>> {
        size_t operator()(const graphix::Id<T> &id) const {
            return hash<typename graphix::Id<T>::value_type>()(id.value());
        }
    };
} // namespace std
