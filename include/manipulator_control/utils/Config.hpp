#ifndef R2D2_CONFIG_HPP
#define R2D2_CONFIG_HPP

#include <vector>
namespace config {

namespace shoulder {
const std::vector<double> coeffs{0.00024, -0.142, 20.9};
const int length{363};        // mm
const int speed{100};         // rad/s
const int angle_offset{0};    // deg
const int angle_tolerance{0}; // deg
} // namespace shoulder

namespace elbow {
const std::vector<double> coeffs{-0.00011, 0.341, -105.2};
const int length{180};        // mm
const int speed{100};         // rad/s
const int angle_offset{0};    // deg
const int angle_tolerance{0}; // deg
} // namespace elbow

namespace ema {
const int target_force{10000}; // N
const double radius0{347};     // mm
} // namespace ema

namespace brush {
const int target_force{15000}; // N
const double radius0{331};     // mm
} // namespace brush

} // namespace config

#endif // R2D2_CONFIG_H
