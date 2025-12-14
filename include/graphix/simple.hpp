#pragma once

#include <string>

namespace graphix {

    class Simple {
      public:
        Simple();
        explicit Simple(const std::string &name);

        void set_name(const std::string &name);
        std::string get_name() const;

        void greet() const;

      private:
        std::string m_name;
    };

} // namespace graphix
