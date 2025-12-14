#include "graphix/factor/values.hpp"

namespace graphix {
    namespace factor {

        // ============================================================================
        // Values implementation
        // ============================================================================

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

        void Values::insert(Key key, double value) {
            if (exists(key)) {
                throw std::runtime_error("Key already exists in Values");
            }
            m_values[key] = std::make_unique<GenericValue<double>>(value);
        }

        double Values::at(Key key) const {
            auto it = m_values.find(key);
            if (it == m_values.end()) {
                throw std::out_of_range("Key not found in Values");
            }

            // Dynamic cast to ensure type safety
            auto *generic_value = dynamic_cast<GenericValue<double> *>(it->second.get());
            if (!generic_value) {
                throw std::runtime_error("Type mismatch: value is not a double");
            }

            return generic_value->value();
        }

        bool Values::exists(Key key) const { return m_values.find(key) != m_values.end(); }

        size_t Values::size() const { return m_values.size(); }

        bool Values::empty() const { return m_values.empty(); }

        void Values::erase(Key key) { m_values.erase(key); }

        void Values::clear() { m_values.clear(); }

    } // namespace factor
} // namespace graphix
