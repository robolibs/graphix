#pragma once

#include "graphix/kernel.hpp"
#include <map>
#include <memory>
#include <stdexcept>

namespace graphix {
    namespace factor {

        // ============================================================================
        // Value: Abstract base class for type-erased values
        // ============================================================================

        class Value {
          public:
            virtual ~Value() = default;

            // Deep copy for cloning Values container
            virtual Value *clone() const = 0;
        };

        // ============================================================================
        // GenericValue<T>: Templated wrapper for actual value storage
        // Step 1a: Initially just support double
        // ============================================================================

        template <typename T> class GenericValue : public Value {
          public:
            explicit GenericValue(const T &value) : m_value(value) {}

            const T &value() const { return m_value; }
            T &value() { return m_value; }

            Value *clone() const override { return new GenericValue<T>(m_value); }

          private:
            T m_value;
        };

        // ============================================================================
        // Values: Type-erased container for variables
        // Step 1a: Support insert/at for double only
        // ============================================================================

        class Values {
          public:
            Values() = default;

            // Copy constructor - deep copy all values
            Values(const Values &other);

            // Assignment operator
            Values &operator=(const Values &other);

            // Move constructor and assignment
            Values(Values &&) = default;
            Values &operator=(Values &&) = default;

            ~Values() = default;

            // Insert a new value (currently double only for Step 1a)
            void insert(Key key, double value);

            // Retrieve a value (currently double only for Step 1a)
            double at(Key key) const;

            // Check if key exists
            bool exists(Key key) const;

            // Get number of values
            size_t size() const;

            // Check if empty
            bool empty() const;

            // Erase a value by key
            void erase(Key key);

            // Clear all values
            void clear();

          private:
            std::map<Key, std::unique_ptr<Value>> m_values;
        };

    } // namespace factor
} // namespace graphix
