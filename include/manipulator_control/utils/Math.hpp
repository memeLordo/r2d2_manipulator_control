#ifndef R2D2_MATH_HPP
#define R2D2_MATH_HPP

// #include "utils/Debug.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace r2d2_math {
template <typename T> constexpr T min(const T a, const T b) {
  return std::min<T>(a, b);
};
template <typename T> constexpr T max(const T a, const T b) {
  return std::max<T>(a, b);
};
template <typename T> constexpr T deg2rad(const T a) {
  return a * M_PI / 180.0;
};
template <typename T> constexpr T sin(const T thetha) {
  return std::sin(deg2rad(thetha));
};
template <typename T> constexpr T sqr(const T a) { return a * a; };
template <typename T> constexpr T sign(const T a) {
  return a ? a > 0 ? 1 : -1 : 0;
};
} // namespace r2d2_math

namespace r2d2_process {

constexpr double K{-100};
template <typename T> const T wrap(T a) {
  return a / K;
  // ROS_DEBUG_STREAM(CYAN("wrap(") << WHITE(a) << CYAN(") -> ") << WHITE(res));
};
template <typename T> const T unwrap(T a) {
  return a * K;
  // ROS_DEBUG_STREAM(CYAN("unwrap(") << WHITE(a) << CYAN(") -> ") <<
  // WHITE(res));
};
template <typename T, typename T2> const T wrap(T2 a) {
  return static_cast<T>(a / K);
  // ROS_DEBUG_STREAM(CYAN("convert wrap(")
  //                  << WHITE(a) << CYAN(") -> ") << WHITE(res));
};
template <typename T, typename T2> const T unwrap(T2 a) {
  return static_cast<T>(a * K);
  // ROS_DEBUG_STREAM(CYAN("convert unwrap(")
  //                  << WHITE(a) << CYAN(") -> ") << WHITE(res));
};

} // namespace r2d2_process

#endif // R2D2_MATH_HPP
