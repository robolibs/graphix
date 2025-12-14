#include "graphix/factor/values.hpp"

namespace graphix {
    namespace factor {

        Values::Values(const Values &other) {
            for (const auto &[key, value_ptr] : other.m_values) {
                m_values[key] = std::unique_ptr<Value>(value_ptr->clone());
            }
        }

        Values &Values::operator=(const Values &other) {
            if (this != &other) {
                m_values.clear();
                for (const auto &[key, value_ptr] : other.m_values) {
                    m_values[key] = std::unique_ptr<Value>(value_ptr->clone());
                }
            }
            return *this;
        }

        bool Values::exists(Key key) const { return m_values.find(key) != m_values.end(); }

        size_t Values::size() const { return m_values.size(); }

        bool Values::empty() const { return m_values.empty(); }

        void Values::erase(Key key) { m_values.erase(key); }

        void Values::clear() { m_values.clear(); }

    } // namespace factor
} // namespace graphix
