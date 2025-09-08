#include "utils/IConfigJson.hpp"
#include "utils/Debug.hpp"
#include <fstream>
#include <ros/package.h>

template <typename T>
std::string IConfigJson<T>::getPath(std::string &&package_name) const {
  return ros::package::getPath(package_name) + "/" + m_dirname + "/";
}
template <typename T> IConfigJson<T>::IConfigJson(const std::string &name) {
  try {
    std::ifstream file(getPath("manipulator_control") + lower(name) + ".json");
    if (!file)
      throw std::runtime_error("Cannot open config file");
    file >> m_json;
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM(RED(e.what()));
    m_json = Json::object();
  }
}
template class IConfigJson<>;
