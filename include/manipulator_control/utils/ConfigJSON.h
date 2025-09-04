#ifndef R2D2_CONFIG_JSON_H
#define R2D2_CONFIG_JSON_H

#include <algorithm>
#include <cctype>
#include <nlohmann/json.hpp>

template <typename T> class ConfigJSON {
  using Json = nlohmann::json;

private:
  struct Config {
    std::string name;
    std::string type;
    T value;
  };
  Json m_json;

public:
  ConfigJSON(const std::string &class_name);
  Json get();
  void parse();
  std::string lower(std::string &str) {
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    return str;
  };
};
#endif // R2D2_CONFIG_JSON_H
