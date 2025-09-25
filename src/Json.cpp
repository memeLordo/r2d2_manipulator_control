#include "r2d2_utils_pkg/Json.hpp"

#include <ros/package.h>

#include "r2d2_utils_pkg/Types.hpp"

using namespace r2d2_type::config;

std::string r2d2_json::getFilePath(const std::string& fileName) {
  const std::string dirName_{"config"};
  const std::string packageName_{"manipulator_control"};
  return {ros::package::getPath(packageName_) + "/" + dirName_ + "/" +
          fileName + ".json"};
}

namespace nlohmann {
template <typename T>
void from_json(const json& j, joint_t<T>& p) {
  j.at("coeffs").get_to(p.coeffs);
  j.at("length").get_to(p.length);
  j.at("speed").get_to(p.speed);
  j.at("angle_offset").get_to(p.angle_offset);
  j.at("angle_tolerance").get_to(p.angle_tolerance);
}
template <typename T>
void from_json(const json& j, manipulator_t<T>& p) {
  j.at("target_force").get_to(p.force_needed);
  j.at("force_tolerance").get_to(p.force_tolerance);
  j.at("init_radius").get_to(p.r0);
}
}  // namespace nlohmann

template <template <typename> class Type, typename T>
IJsonConfigMap<Type, T>::IJsonConfigMap(const std::string& fileName)
    : IJsonConfig<T>(fileName) {
  for (auto& el : this->m_json.items())
    m_paramsMap[el.key()] = el.value().template get<Type<T>>();
}

template class IJsonConfigMap<joint_t>;
template class IJsonConfigMap<manipulator_t>;
