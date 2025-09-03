#ifndef R2D2_CONFIG_JSON_H
#define R2D2_CONFIG_JSON_H

#include <nlohmann/json.hpp>

template <typename T> class ConfigJSON {
  using Json = nlohmann::json;

private:
  static Json m_json;

public:
  ConfigJSON(const std::string &path);
  static Json s_get() { return m_json; }
  static void s_parse();
};

#endif // R2D2_CONFIG_JSON_H
