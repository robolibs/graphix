#pragma once

#include "graphix/kernel.hpp"
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

namespace graphix {
    namespace factor {

        // Abstract base class for type-erased values
        class Value {
          public:
            virtual ~Value() = default;
            virtual Value *clone() const = 0;
        };

        // Template wrapper for actual value storage
        template <typename T> class GenericValue : public Value {
          public:
            explicit GenericValue(const T &value) : m_value(value) {}

            const T &value() const { return m_value; }
            T &value() { return m_value; }

            Value *clone() const override { return new GenericValue<T>(m_value); }

          private:
            T m_value;
        };

        // Type-erased container for variables
        class Values {
          public:
            Values() = default;

            inline Values(const Values &other) {
                for (const auto &[key, value_ptr] : other.m_values) {
                    m_values[key] = std::unique_ptr<Value>(value_ptr->clone());
                }
            }

            inline Values &operator=(const Values &other) {
                if (this != &other) {
                    m_values.clear();
                    for (const auto &[key, value_ptr] : other.m_values) {
                        m_values[key] = std::unique_ptr<Value>(value_ptr->clone());
                    }
                }
                return *this;
            }

            Values(Values &&) = default;
            Values &operator=(Values &&) = default;

            ~Values() = default;

            // Insert a value of any type
            template <typename T> void insert(Key key, const T &value) {
                if (exists(key)) {
                    throw std::runtime_error("Key already exists in Values");
                }
                m_values[key] = std::make_unique<GenericValue<T>>(value);
            }

            // Retrieve a value with type checking
            template <typename T> const T &at(Key key) const {
                auto it = m_values.find(key);
                if (it == m_values.end()) {
                    throw std::out_of_range("Key not found in Values");
                }

                auto *generic_value = dynamic_cast<GenericValue<T> *>(it->second.get());
                if (!generic_value) {
                    throw std::runtime_error("Type mismatch: requested type does not match stored type");
                }

                return generic_value->value();
            }

            inline bool exists(Key key) const { return m_values.find(key) != m_values.end(); }
            inline size_t size() const { return m_values.size(); }
            inline bool empty() const { return m_values.empty(); }
            inline void erase(Key key) { m_values.erase(key); }
            inline void clear() { m_values.clear(); }

            // Get all keys
            inline std::vector<Key> keys() const {
                std::vector<Key> result;
                result.reserve(m_values.size());
                for (const auto &[key, _] : m_values) {
                    result.push_back(key);
                }
                return result;
            }

            // Iterator support for range-based loops
            using const_iterator = std::map<Key, std::unique_ptr<Value>>::const_iterator;

            const_iterator begin() const { return m_values.begin(); }
            const_iterator end() const { return m_values.end(); }

          private:
            std::map<Key, std::unique_ptr<Value>> m_values;
        };

    } // namespace factor
} // namespace graphix
