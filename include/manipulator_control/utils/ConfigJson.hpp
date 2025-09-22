#ifndef R2D2_CONFIG_JSON_HPP
#define R2D2_CONFIG_JSON_HPP

#include <fstream>
#include <nlohmann/json.hpp>

namespace r2d2_json {
inline std::string getFilePath(const std::string& fileName);
}  // namespace r2d2_json

template <typename T = double>
class IConfigJson {
 protected:
  nlohmann::json m_json;

 protected:
  IConfigJson(const std::string& fileName) {
    std::ifstream file(r2d2_json::getFilePath(fileName));
    if (!file)
      throw std::runtime_error("File \"" + fileName + ".json\" not found!");
    file >> m_json;
  };

 protected:
  template <typename U = T>
  U getParam(const std::string& key) const {
    if (!m_json.contains(key))
      throw std::runtime_error("Parameter \"" + key + "\" not found!");
    return m_json.at(key).get<U>();
  };
};

template <template <typename> class Type, typename T = double>
class IConfigJsonMap : protected IConfigJson<T> {
 private:
  std::unordered_map<std::string, Type<T>> m_paramsMap;

 protected:
  IConfigJsonMap(const std::string& fileName);

 protected:
  Type<T> getParams(const std::string& key) const {
    auto it{m_paramsMap.find(key)};
    if (it == m_paramsMap.end())
      throw std::runtime_error("Object \"" + key + "\" not found!");
    return it->second;
  };
};
#endif  // R2D2_CONFIG_JSON_HPP
