#include "r2d2_utils_pkg/Math.hpp"

#include "r2d2_utils_pkg/ConfigJson.hpp"

namespace r2d2_process {
IConfigJson<> convertConfig{"convert_rate"};
const double Angle::s_convertRatio{convertConfig.getParam("angle")};
const double Force::s_convertRatio{convertConfig.getParam("force")};
}  // namespace r2d2_process
