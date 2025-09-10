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
protected:
  nlohmann::json m_json;
  IConfigJson(const std::string &name);

  T getParam(const std::string &key) const {
    if (m_json.contains(key))
      return m_json.at(key).get<T>();
    return T{};
  };
  std::vector<T> getVector(const std::string &key) const {
    if (m_json.contains(key))
      return m_json.at(key).get<std::vector<T>>();
    return std::vector<T>{};
  };
};

template <typename U> class IConfigJsonMap : private IConfigJson<U> {
private:
  std::unordered_map<std::string, U> m_paramsMap;

protected:
  IConfigJsonMap(const std::string &fileName);
  U getParams(const std::string &key) const;
};
#endif // R2D2_CONFIG_JSON_HPP
