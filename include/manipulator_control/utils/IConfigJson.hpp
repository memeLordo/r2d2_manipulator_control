#ifndef R2D2_CONFIG_JSON_HPP
#define R2D2_CONFIG_JSON_HPP

#include <fstream>
#include <nlohmann/json.hpp>

namespace r2d2_json {
inline std::string getFilePath(const std::string &fileName);

inline std::string lower(std::string str) {
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
  return str;
};
}  // namespace r2d2_json

template <typename T = double>
class IConfigJson {
 protected:
  nlohmann::json m_json;

 protected:
  IConfigJson(const std::string &fileName) {
    std::ifstream file(r2d2_json::getFilePath(fileName));
    if (!file)
      throw std::runtime_error("File " + fileName + ".json not found!");
    file >> m_json;
  };

 protected:
  T getParam(const std::string &key) const {
    if (m_json.contains(key)) return m_json.at(key).get<T>();
    return T{};
  };
  std::vector<T> getVector(const std::string &key) const {
    if (m_json.contains(key)) return m_json.at(key).get<std::vector<T>>();
    return std::vector<T>{};
  };
};

template <template <typename> class Type, typename U = double>
class IConfigJsonMap : private IConfigJson<Type<U>> {
 private:
  std::unordered_map<std::string, Type<U>> m_paramsMap;

 protected:
  IConfigJsonMap(const std::string &fileName);
  Type<U> getParams(const std::string &key) const {
    auto it{m_paramsMap.find(key)};
    if (it != m_paramsMap.end()) {
      return it->second;
    }
    return Type<U>{};
  };
};
#endif  // R2D2_CONFIG_JSON_HPP
