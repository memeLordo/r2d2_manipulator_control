#include "utils/IConfigJson.hpp"
#include "utils/Debug.hpp"
#include <fstream>
#include <ros/package.h>

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
template class IConfigJson<>;
