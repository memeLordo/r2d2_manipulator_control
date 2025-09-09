#ifndef R2D2_CONFIG_JSON_HPP
#define R2D2_CONFIG_JSON_HPP

#include <algorithm>
#include <cctype>
#include <nlohmann/json.hpp>
#include <vector>

namespace r2d2_json {
inline std::string getPath(const std::string &&package_name,
                           const std::string &&dirname = "config");

inline std::string lower(std::string name) {
  std::transform(name.begin(), name.end(), name.begin(), ::tolower);
  return name;
};
} // namespace r2d2_json

template <typename T = double> class IConfigJson {
private:
  nlohmann::json m_json;

protected:
  IConfigJson();
  IConfigJson(const std::string &name);

  T getParam(const std::string &key) const {
    if (m_json.contains(key))
      return m_json.at(key).get<T>();
    return T{0};
  };
  std::vector<T> getVector(const std::string &key) const {
    if (m_json.contains(key))
      return m_json.at(key).get<std::vector<T>>();
    return std::vector<T>{0};
  };
};
#endif // R2D2_CONFIG_JSON_HPP
