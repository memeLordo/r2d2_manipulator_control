#include "r2d2_utils_pkg/Math.hpp"

#include "r2d2_utils_pkg/Json.hpp"

using namespace r2d2_process;

IJsonConfig<true> jc{"convert_rate"};
const double Angle::s_convertRatio{jc.getParam("angle")};
const double Force::s_convertRatio{jc.getParam("force")};
