#include "graphix/simple.hpp"
#include <iostream>

int main() {
    std::cout << "Graphix - A modern C++ graph library" << std::endl;
    std::cout << "=====================================" << std::endl;

    graphix::Simple simple;
    simple.greet();

    return 0;
}
