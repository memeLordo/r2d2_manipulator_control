#include "utils/IConfigJson.hpp"
#include "utils/Debug.hpp"
#include <fstream>
#include <ros/package.h>

template <typename T>
IConfigJson<T>::IConfigJson(const std::string &name)
    : m_path{ros::package::getPath("manipulator_control") + "/" + m_dirname +
             "/"} {
  try {
    std::ifstream file(m_path + lower(name) + ".json");
    if (!file)
      throw std::runtime_error("Cannot open config file");
    file >> m_json;
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM(RED(e.what()));
    m_json = Json::object();
  }
}
template class IConfigJson<>;
