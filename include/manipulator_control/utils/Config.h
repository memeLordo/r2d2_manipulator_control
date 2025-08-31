#ifndef R2D2_CONFIG_H
#define R2D2_CONFIG_H

#include <array>
#include <string>
namespace config {

namespace shoulder {
const std::string INPUT_NODE = "/shoulder_input";
const std::string OUTPUT_NODE = "/shoulder_output";

const std::array<double, 3> coeffs{0.00024, -0.142, 20.9};
const int length{363}; // mm
const int speed{100};
// const double angle_treshold{5};
} // namespace shoulder

namespace elbow {
const std::string INPUT_NODE = "/elbow_input";
const std::string OUTPUT_NODE = "/elbow_output";

const std::array<double, 3> coeffs{-0.00011, 0.341, -105.2};
const int length{180}; // mm
const int speed{100};
// const double angle_treshold{5};
} // namespace elbow

namespace pipe {
const std::string OUTPUT_NODE = "/pipe_output";
}

namespace payload {
const std::string OUTPUT_NODE = "/payload_output";
}

} // namespace config

#endif // R2D2_CONFIG_H
