#include "utils/IConfigJson.hpp"
#include "utils/Debug.hpp"
#include "utils/Types.hpp"
#include <fstream>
#include <ros/package.h>
// #include <unordered_map>

using namespace r2d2_type;

std::string r2d2_json::getPath(const std::string &&package_name,
                               const std::string &&dirname) {
  return ros::package::getPath(package_name) + "/" + dirname + "/";
}
template <typename T> IConfigJson<T>::IConfigJson(const std::string &name) {
  using namespace r2d2_json;
  try {
    std::ifstream file(getPath("manipulator_control") + lower(name) + ".json");
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
void from_json(const nlohmann::json &j, manipulator16_t<T> &p) {
  p.force_needed = j.at("target_force").get<T>();
  p.force_tolerance = j.at("force_tolerance").get<T>();
  p.r0 = j.at("init_radius").get<T>();
}
} // namespace nlohmann

template <typename U> class IConfigJson<manipulator16_t<U>> {
private:
  nlohmann::json m_json;

protected:
  manipulator16_t<U> m_params;

public:
  IConfigJson(const std::string &name) {
    using namespace r2d2_json;
    try {
      std::ifstream file(getPath("manipulator_control") + lower(name) +
                         ".json");
      if (!file)
        throw std::runtime_error("Cannot open config file!");
      file >> m_json;

      for (auto &el : m_json.items()) {
        m_paramsMap[el.key()] = el.value().get<manipulator16_t<U>>();
      }
    } catch (const std::exception &e) {
      ROS_ERROR_STREAM(RED(e.what()));
      m_json = nlohmann::json::object();
    }
  };
  manipulator16_t<U> getParams(r2d2_state::NozzleType nozzleType) const {
    auto it{m_paramsMap.find(r2d2_state::toString(nozzleType))};
    if (it != m_paramsMap.end()) {
      return it->second;
    }
    ROS_ERROR_STREAM(RED("Invalid nozzle type!"));
    return manipulator16_t<U>{};
  };
};

template class IConfigJson<manipulator16_t<>>;
template class IConfigJson<>;
