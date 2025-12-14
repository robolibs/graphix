#pragma once

#include "graphix/factor/factor.hpp"
#include "graphix/factor/values.hpp"

namespace graphix {
    namespace factor {

        // NonlinearFactor: Factor with error computation
        // Extends Factor (structure) with math (error function)
        class NonlinearFactor : public Factor {
          public:
            using Factor::Factor; // Inherit constructors

            virtual ~NonlinearFactor() = default;

            // Compute error given variable values
            // Returns scalar error (0.5 * weighted squared residual)
            // Used for optimization objective: minimize sum of errors
            virtual double error(const Values &values) const = 0;
        };

    } // namespace factor
} // namespace graphix
