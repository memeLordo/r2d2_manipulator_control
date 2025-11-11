#include "r2d2_utils_pkg/Json.hpp"

#include <ros/package.h>

#include "r2d2_utils_pkg/Types.hpp"

using namespace r2d2_type::config;

/**
 * @brief Gets the full file path for a configuration file.
 * @param fileName The name of the configuration file (without extension)
 * @return The full path to the configuration file
 * @details Constructs the path as: <package_path>/config/<fileName>.json
 */
std::string r2d2_json::getFilePath(std::string_view fileName) noexcept {
  const std::string dirName_{"config"};
  const std::string packageName_{"manipulator_control"};
  return {ros::package::getPath(packageName_) + "/" + dirName_ + "/" +
          std::string{fileName} + ".json"};
}

namespace nlohmann {
/**
 * @brief Deserializes a joint_t configuration from JSON.
 * @tparam T Numeric type for the configuration values
 * @param j The JSON object to deserialize from
 * @param p The joint_t object to populate
 * @details Extracts coefficients, length, speed, angle_offset, and
 * angle_tolerance from JSON.
 */
template <typename T>
void from_json(const json& j, joint_t<T>& p) {
  j.at("coeffs").get_to(p.coeffs);
  j.at("length").get_to(p.length);
  j.at("speed").get_to(p.speed);
  j.at("angle_offset").get_to(p.angle_offset);
  j.at("angle_tolerance").get_to(p.angle_tolerance);
}
/**
 * @brief Deserializes a nozzle_t configuration from JSON.
 * @tparam T Numeric type for the configuration values
 * @param j The JSON object to deserialize from
 * @param p The nozzle_t object to populate
 * @details Extracts target_force, force_tolerance, and init_radius from JSON.
 */
template <typename T>
void from_json(const json& j, nozzle_t<T>& p) {
  j.at("target_force").get_to(p.force_needed);
  j.at("force_tolerance").get_to(p.force_tolerance);
  j.at("init_radius").get_to(p.r0);
}
}  // namespace nlohmann

/**
 * @brief Constructs an IJsonConfigMap and loads all parameters from JSON.
 * @tparam Type The configuration type template
 * @tparam T Numeric type for the configuration values
 * @param fileName The name of the JSON configuration file
 * @details Loads the JSON file and deserializes all entries into a map of
 * configuration objects.
 */
template <template <typename> class Type, typename T>
IJsonConfigMap<Type, T>::IJsonConfigMap(std::string_view fileName)
    : IJsonConfig<>(fileName) {
  for (const auto& [key, value] : this->m_json.items()) {
    m_paramsMap[key] = value.template get<Type<T>>();
  }
}

template class IJsonConfigMap<joint_t>;
template class IJsonConfigMap<nozzle_t>;
