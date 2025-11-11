#include "r2d2_utils_pkg/Math.hpp"

#include "r2d2_utils_pkg/Json.hpp"

using namespace r2d2_process;

/**
 * @brief Global configuration object for loading conversion rate parameters.
 * @details Loads the "convert_rate.json" configuration file to access angle and force conversion ratios.
 */
IJsonConfig<true> jc{"convert_rate"};

/**
 * @brief Global angle conversion ratio loaded from configuration.
 * @details Used to convert between different angle representations or units.
 */
const double config::g_angleRatio{jc.getParam("angle")};

/**
 * @brief Global force conversion ratio loaded from configuration.
 * @details Used to convert between different force representations or units.
 */
const double config::g_forceRatio{jc.getParam("force")};
