#ifndef R2D2_CONFIG_JSON_HPP
#define R2D2_CONFIG_JSON_HPP

#include <algorithm>
#include <cctype>
#include <nlohmann/json.hpp>
#include <vector>

template <typename T = double> class IConfigJson {
  using Json = nlohmann::json;

private:
  const std::string m_dirname{"config"};

  Json m_json;

  std::string getPath(std::string &&package_name) const;

protected:
  IConfigJson(const std::string &name);

  std::string lower(std::string name) const {
    std::transform(name.begin(), name.end(), name.begin(), ::tolower);
    return name;
  };
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
