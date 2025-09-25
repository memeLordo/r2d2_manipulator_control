#include "r2d2_utils_pkg/Math.hpp"

#include "r2d2_utils_pkg/Json.hpp"

namespace r2d2_process {
IJsonConfig<> jc{"convert_rate"};
const double Angle::s_convertRatio{jc.getParam("angle")};
const double Force::s_convertRatio{jc.getParam("force")};
}  // namespace r2d2_process
