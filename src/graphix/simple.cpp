#include "graphix/simple.hpp"
#include <iostream>

namespace graphix {

    Simple::Simple() : m_name("World") {}

    Simple::Simple(const std::string &name) : m_name(name) {}

    void Simple::set_name(const std::string &name) { m_name = name; }

    std::string Simple::get_name() const { return m_name; }

    void Simple::greet() const { std::cout << "Hello, " << m_name << "!" << std::endl; }

} // namespace graphix
