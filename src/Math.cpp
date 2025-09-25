#include "r2d2_utils_pkg/Math.hpp"

#include "r2d2_utils_pkg/Json.hpp"

namespace r2d2_process {
IJsonConfig<> json{"convert_rate"};
const double Angle::s_convertRatio{json.getParam("angle")};
const double Force::s_convertRatio{json.getParam("force")};
}  // namespace r2d2_process
