#ifndef R2D2_STRING_HPP
#define R2D2_STRING_HPP

#include <algorithm>
#include <cassert>
#include <string>

namespace r2d2_string {
inline std::string lower(std::string str) {
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
  return str;
};
inline std::string lower(std::string str, size_t from) {
  assert(from >= 0 && from <= str.length());
  auto begin_{str.begin() + from};
  auto end_{str.end()};
  std::transform(begin_, end_, begin_, ::tolower);
  return str;
};
inline std::string lower(std::string str, size_t from, size_t to) {
  assert(from >= 0 && from <= str.length());
  assert(to >= from && to <= str.length());
  auto begin_{str.begin() + from};
  auto end_{str.begin() + to};
  std::transform(begin_, end_, begin_, ::tolower);
  return str;
};

inline std::string upper(std::string str) {
  std::transform(str.begin(), str.end(), str.begin(), ::toupper);
  return str;
};
inline std::string upper(std::string str, size_t from) {
  assert(from >= 0 && from <= str.length());
  auto begin_{str.begin() + from};
  auto end_{str.end()};
  std::transform(begin_, end_, begin_, ::toupper);
  return str;
};
inline std::string upper(std::string str, size_t from, size_t to) {
  assert(from >= 0 && from <= str.length());
  assert(to >= from && to <= str.length());
  auto begin_{str.begin() + from};
  auto end_{str.begin() + to};
  std::transform(begin_, end_, begin_, ::toupper);
  return str;
};
}  // namespace r2d2_string

#endif  // R2D2_STRING_HPP
