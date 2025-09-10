#include "utils/IConfigJson.hpp"
#include "utils/Debug.hpp"
#include "utils/Types.hpp"
#include <fstream>
#include <ros/package.h>

using namespace r2d2_type;

std::string r2d2_json::getPath(const std::string &packageName,
                               const std::string &dirName) {
  return ros::package::getPath(packageName) + "/" + dirName + "/";
}
template <typename T> IConfigJson<T>::IConfigJson(const std::string &fileName) {
  using namespace r2d2_json;
  try {
    std::ifstream file(getPath("manipulator_control") + fileName + ".json");
    if (!file)
      throw std::runtime_error("Cannot open config file");
    file >> m_json;
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM(RED(e.what()));
    m_json = nlohmann::json::object();
  }
}
namespace nlohmann {
template <typename T>
void from_json(const json &j, config::manipulator_t<T> &p) {
  j.at("target_force").get_to(p.force_needed);
  j.at("force_tolerance").get_to(p.force_tolerance);
  j.at("init_radius").get_to(p.r0);
}
} // namespace nlohmann

template <typename U>
IConfigJsonMap<U>::IConfigJsonMap(const std::string &fileName)
    : IConfigJson<U>(fileName) {
  for (auto &el : this->m_json.items()) {
    m_paramsMap[el.key()] = el.value().template get<U>();
  }
};

template <typename U>
U IConfigJsonMap<U>::getParams(const std::string &key) const {
  auto it{m_paramsMap.find(key)};
  if (it != m_paramsMap.end()) {
    return it->second;
  }
  ROS_ERROR_STREAM(RED("Invalid key!"));
  return U{};
};

template class IConfigJson<>;
template class IConfigJsonMap<config::manipulator_t<>>;
