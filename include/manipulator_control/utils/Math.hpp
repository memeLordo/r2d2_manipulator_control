#ifndef R2D2_MATH_HPP
#define R2D2_MATH_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace r2d2_math {
template <typename T>
constexpr T min(const T a, const T b) {
  return std::min<T>(a, b);
};
template <typename T>
constexpr T max(const T a, const T b) {
  return std::max<T>(a, b);
};
template <typename T>
constexpr T deg2rad(const T a) {
  return a * M_PI / 180.0;
};
template <typename T>
constexpr T sin(const T thetha) {
  return std::sin(deg2rad(thetha));
};
template <typename T>
constexpr T sqr(const T a) {
  return a * a;
};
template <typename T>
constexpr int8_t sign(const T a) {
  return a ? a > 0 ? 1 : -1 : 0;
};
}  // namespace r2d2_math

namespace r2d2_process {
constexpr double K{-100};

template <typename T>
constexpr T wrap(const T a) {
  return a / K;
};
template <typename T>
constexpr T unwrap(const T a) {
  return a * K;
};
template <typename T, typename T2>
constexpr T wrap(const T2 a) {
  return static_cast<T>(a / K);
};
template <typename T, typename T2>
constexpr T unwrap(const T2 a) {
  return static_cast<T>(a * K);
};
}  // namespace r2d2_process

#endif  // R2D2_MATH_HPP
