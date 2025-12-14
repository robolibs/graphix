#pragma once

#include "graphix/vertex/graph.hpp"
#include <memory>
#include <stdexcept>
#include <unordered_map>

namespace graphix {
    namespace vertex {

        // ============================================================================
        // Property Map Base Class (for type erasure and polymorphism)
        // ============================================================================

        template <typename Key, typename Value> class PropertyMap {
          public:
            virtual ~PropertyMap() = default;

            // Core property map operations
            virtual Value get(const Key &k) const = 0;
            virtual void put(const Key &k, const Value &v) = 0;
            virtual bool contains(const Key &k) const = 0;
            virtual void clear() = 0;
            virtual size_t size() const = 0;
        };

        // ============================================================================
        // Associative Property Map (hash map based)
        // ============================================================================
        // Stores properties externally in a hash map
        // Most flexible - can add properties to any vertex/edge dynamically

        template <typename Key, typename Value> class AssociativePropertyMap : public PropertyMap<Key, Value> {
          public:
            AssociativePropertyMap() = default;
            explicit AssociativePropertyMap(const Value &default_value)
                : m_default(default_value), m_has_default(true) {}

            Value get(const Key &k) const override {
                auto it = m_storage.find(k);
                if (it != m_storage.end()) {
                    return it->second;
                }
                if (m_has_default) {
                    return m_default;
                }
                throw std::out_of_range("Key not found in property map");
            }

            void put(const Key &k, const Value &v) override { m_storage[k] = v; }

            bool contains(const Key &k) const override { return m_storage.find(k) != m_storage.end(); }

            void clear() override { m_storage.clear(); }

            size_t size() const override { return m_storage.size(); }

            // Additional methods
            void erase(const Key &k) { m_storage.erase(k); }

            void set_default(const Value &v) {
                m_default = v;
                m_has_default = true;
            }

            // Iterator support
            auto begin() { return m_storage.begin(); }
            auto end() { return m_storage.end(); }
            auto begin() const { return m_storage.begin(); }
            auto end() const { return m_storage.end(); }

          private:
            std::unordered_map<Key, Value> m_storage;
            Value m_default{};
            bool m_has_default = false;
        };

        // ============================================================================
        // Vector Property Map (index-based, faster for dense vertex sets)
        // ============================================================================
        // Uses vertex/edge ID as index into a vector
        // Faster access but requires contiguous IDs

        template <typename Value> class VectorPropertyMap : public PropertyMap<size_t, Value> {
          public:
            VectorPropertyMap() = default;
            explicit VectorPropertyMap(size_t initial_size, const Value &default_value = Value())
                : m_storage(initial_size, default_value), m_default(default_value) {}

            Value get(const size_t &k) const override {
                if (k < m_storage.size()) {
                    return m_storage[k];
                }
                return m_default;
            }

            void put(const size_t &k, const Value &v) override {
                if (k >= m_storage.size()) {
                    m_storage.resize(k + 1, m_default);
                }
                m_storage[k] = v;
            }

            bool contains(const size_t &k) const override { return k < m_storage.size(); }

            void clear() override { m_storage.clear(); }

            size_t size() const override { return m_storage.size(); }

            // Additional methods
            void reserve(size_t capacity) { m_storage.reserve(capacity); }

            void resize(size_t new_size) { m_storage.resize(new_size, m_default); }

            void set_default(const Value &v) { m_default = v; }

          private:
            std::vector<Value> m_storage;
            Value m_default{};
        };

        // ============================================================================
        // Constant Property Map (always returns same value)
        // ============================================================================
        // Useful for default properties or testing

        template <typename Key, typename Value> class ConstantPropertyMap : public PropertyMap<Key, Value> {
          public:
            explicit ConstantPropertyMap(const Value &value) : m_value(value) {}

            Value get(const Key &) const override { return m_value; }

            void put(const Key &, const Value &) override {
                // Constant map ignores writes
            }

            bool contains(const Key &) const override {
                return true; // Always contains any key
            }

            void clear() override {
                // Nothing to clear
            }

            size_t size() const override {
                return 0; // Conceptually infinite
            }

          private:
            Value m_value;
        };

        // ============================================================================
        // Identity Property Map (returns key as value)
        // ============================================================================
        // Useful when the property IS the descriptor itself

        template <typename Key> class IdentityPropertyMap : public PropertyMap<Key, Key> {
          public:
            IdentityPropertyMap() = default;

            Key get(const Key &k) const override { return k; }

            void put(const Key &, const Key &) override {
                // Identity map ignores writes
            }

            bool contains(const Key &) const override { return true; }

            void clear() override {
                // Nothing to clear
            }

            size_t size() const override { return 0; }
        };

        // ============================================================================
        // Free Function Interface (Boost.Graph compatible)
        // ============================================================================

        // get(property_map, key) - read property
        template <typename Key, typename Value, typename K> Value get(const PropertyMap<Key, Value> &pmap, const K &k) {
            return pmap.get(k);
        }

        template <typename Key, typename Value, typename K>
        Value get(const std::shared_ptr<PropertyMap<Key, Value>> &pmap, const K &k) {
            return pmap->get(k);
        }

        // put(property_map, key, value) - write property
        template <typename Key, typename Value, typename K, typename V>
        void put(PropertyMap<Key, Value> &pmap, const K &k, const V &v) {
            pmap.put(k, v);
        }

        template <typename Key, typename Value, typename K, typename V>
        void put(std::shared_ptr<PropertyMap<Key, Value>> &pmap, const K &k, const V &v) {
            pmap->put(k, v);
        }

        // Overloads for concrete property map types (AssociativePropertyMap)
        template <typename Key, typename Value, typename K>
        Value get(const AssociativePropertyMap<Key, Value> &pmap, const K &k) {
            return pmap.get(k);
        }

        template <typename Key, typename Value, typename K>
        Value get(const std::shared_ptr<AssociativePropertyMap<Key, Value>> &pmap, const K &k) {
            return pmap->get(k);
        }

        template <typename Key, typename Value, typename K, typename V>
        void put(AssociativePropertyMap<Key, Value> &pmap, const K &k, const V &v) {
            pmap.put(k, v);
        }

        template <typename Key, typename Value, typename K, typename V>
        void put(std::shared_ptr<AssociativePropertyMap<Key, Value>> &pmap, const K &k, const V &v) {
            pmap->put(k, v);
        }

        // Overloads for VectorPropertyMap (Key is always size_t)
        template <typename Value, typename K> Value get(const VectorPropertyMap<Value> &pmap, const K &k) {
            return pmap.get(k);
        }

        template <typename Value, typename K>
        Value get(const std::shared_ptr<VectorPropertyMap<Value>> &pmap, const K &k) {
            return pmap->get(k);
        }

        template <typename Value, typename K, typename V>
        void put(VectorPropertyMap<Value> &pmap, const K &k, const V &v) {
            pmap.put(k, v);
        }

        template <typename Value, typename K, typename V>
        void put(std::shared_ptr<VectorPropertyMap<Value>> &pmap, const K &k, const V &v) {
            pmap->put(k, v);
        }

        template <typename Value> Value get(const std::shared_ptr<VectorPropertyMap<Value>> &pmap, const size_t &k) {
            return pmap->get(k);
        }

        template <typename Value> void put(VectorPropertyMap<Value> &pmap, const size_t &k, const Value &v) {
            pmap.put(k, v);
        }

        template <typename Value>
        void put(std::shared_ptr<VectorPropertyMap<Value>> &pmap, const size_t &k, const Value &v) {
            pmap->put(k, v);
        }

        // Overloads for ConstantPropertyMap
        template <typename Key, typename Value, typename K>
        Value get(const ConstantPropertyMap<Key, Value> &pmap, const K &k) {
            return pmap.get(k);
        }

        template <typename Key, typename Value, typename K>
        Value get(const std::shared_ptr<ConstantPropertyMap<Key, Value>> &pmap, const K &k) {
            return pmap->get(k);
        }

        template <typename Key, typename Value, typename K, typename V>
        void put(ConstantPropertyMap<Key, Value> &pmap, const K &k, const V &v) {
            pmap.put(k, v);
        }

        template <typename Key, typename Value, typename K, typename V>
        void put(std::shared_ptr<ConstantPropertyMap<Key, Value>> &pmap, const K &k, const V &v) {
            pmap->put(k, v);
        }

        // Overloads for IdentityPropertyMap
        template <typename Key, typename K> Key get(const IdentityPropertyMap<Key> &pmap, const K &k) {
            return pmap.get(k);
        }

        template <typename Key, typename K> Key get(const std::shared_ptr<IdentityPropertyMap<Key>> &pmap, const K &k) {
            return pmap->get(k);
        }

        template <typename Key, typename K, typename V>
        void put(IdentityPropertyMap<Key> &pmap, const K &k, const V &v) {
            pmap.put(k, v);
        }

        template <typename Key, typename K, typename V>
        void put(std::shared_ptr<IdentityPropertyMap<Key>> &pmap, const K &k, const V &v) {
            pmap->put(k, v);
        }

        // ============================================================================
        // Factory Functions (convenience)
        // ============================================================================

        // Create an associative property map
        template <typename Key, typename Value>
        std::shared_ptr<AssociativePropertyMap<Key, Value>> make_associative_property_map() {
            return std::make_shared<AssociativePropertyMap<Key, Value>>();
        }

        template <typename Key, typename Value>
        std::shared_ptr<AssociativePropertyMap<Key, Value>> make_associative_property_map(const Value &default_value) {
            return std::make_shared<AssociativePropertyMap<Key, Value>>(default_value);
        }

        // Create a vector property map
        template <typename Value>
        std::shared_ptr<VectorPropertyMap<Value>> make_vector_property_map(size_t size = 0,
                                                                           const Value &default_value = Value()) {
            return std::make_shared<VectorPropertyMap<Value>>(size, default_value);
        }

        // Create a constant property map
        template <typename Key, typename Value>
        std::shared_ptr<ConstantPropertyMap<Key, Value>> make_constant_property_map(const Value &value) {
            return std::make_shared<ConstantPropertyMap<Key, Value>>(value);
        }

        // Create an identity property map
        template <typename Key> std::shared_ptr<IdentityPropertyMap<Key>> make_identity_property_map() {
            return std::make_shared<IdentityPropertyMap<Key>>();
        }

        // ============================================================================
        // Composite Property Map (multiple properties together)
        // ============================================================================
        // Manages a collection of property maps for a graph

        template <typename VertexProperty = void, typename EdgeProperty = void> class CompositePropertyMap {
          public:
            using VertexId = size_t;
            using EdgeId = size_t;

            CompositePropertyMap() = default;

            // Register a named vertex property map (accepts both base and derived types)
            template <typename PropertyMapType>
            void add_vertex_property(const std::string &name, std::shared_ptr<PropertyMapType> pmap) {
                m_vertex_properties[name] = std::static_pointer_cast<void>(pmap);
            }

            // Register a named edge property map (accepts both base and derived types)
            template <typename PropertyMapType>
            void add_edge_property(const std::string &name, std::shared_ptr<PropertyMapType> pmap) {
                m_edge_properties[name] = std::static_pointer_cast<void>(pmap);
            }

            // Get a vertex property map by name
            template <typename Value>
            std::shared_ptr<PropertyMap<VertexId, Value>> get_vertex_property(const std::string &name) {
                auto it = m_vertex_properties.find(name);
                if (it != m_vertex_properties.end()) {
                    return std::static_pointer_cast<PropertyMap<VertexId, Value>>(it->second);
                }
                return nullptr;
            }

            // Get an edge property map by name
            template <typename Value>
            std::shared_ptr<PropertyMap<EdgeId, Value>> get_edge_property(const std::string &name) {
                auto it = m_edge_properties.find(name);
                if (it != m_edge_properties.end()) {
                    return std::static_pointer_cast<PropertyMap<EdgeId, Value>>(it->second);
                }
                return nullptr;
            }

            // Check if property exists
            bool has_vertex_property(const std::string &name) const {
                return m_vertex_properties.find(name) != m_vertex_properties.end();
            }

            bool has_edge_property(const std::string &name) const {
                return m_edge_properties.find(name) != m_edge_properties.end();
            }

            // Remove property maps
            void remove_vertex_property(const std::string &name) { m_vertex_properties.erase(name); }

            void remove_edge_property(const std::string &name) { m_edge_properties.erase(name); }

            // Clear all properties
            void clear() {
                m_vertex_properties.clear();
                m_edge_properties.clear();
            }

          private:
            std::unordered_map<std::string, std::shared_ptr<void>> m_vertex_properties;
            std::unordered_map<std::string, std::shared_ptr<void>> m_edge_properties;
        };

    } // namespace vertex
} // namespace graphix
