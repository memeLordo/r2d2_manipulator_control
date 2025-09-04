#ifndef R2D2_CONFIG_HPP
#define R2D2_CONFIG_HPP

#include <vector>
namespace config {

namespace shoulder {

const std::vector<double> coeffs{0.00024, -0.142, 20.9};
const int length{363}; // mm
const int speed{100};
// const double angle_treshold{5};
} // namespace shoulder

namespace elbow {

const std::vector<double> coeffs{-0.00011, 0.341, -105.2};
const int length{180}; // mm
const int speed{100};
// const double angle_treshold{5};
} // namespace elbow

} // namespace config

#endif // R2D2_CONFIG_H
