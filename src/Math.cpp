#include "r2d2_utils_pkg/Math.hpp"

#include "r2d2_utils_pkg/Json.hpp"

using namespace r2d2_process;

IJsonConfig<true> jc{"convert_rate"};
const double config::g_angleRatio{jc.getParam("angle")};
const double config::g_forceRatio{jc.getParam("force")};
