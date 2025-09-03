#ifndef R2D2_MATH_HPP
#define R2D2_MATH_HPP

#include "utils/Debug.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ros/console.h>

namespace r2d2_math {

template <typename T> constexpr T deg2rad(const T a) { return a * M_PI / 180; };
template <typename T> constexpr T abs(const T a) { return std::abs(a); };
template <typename T> constexpr T min(const T a, const T b) {
  return std::min<T>(a, b);
};
template <typename T> constexpr T max(const T a, const T b) {
  return std::max<T>(a, b);
};
template <typename T> constexpr T sin(const T thetha) {
  return std::sin(deg2rad(thetha));
};

} // namespace r2d2_math

namespace r2d2_process {

constexpr double K{-100};
template <typename T> const T wrap(T a) {
  T res = a / K;
  ROS_DEBUG_STREAM(CYAN("wrap(") << WHITE(a) << CYAN(") -> ") << WHITE(res));
  return res;
};
template <typename T> const T unwrap(T a) {
  T res = a * K;
  ROS_DEBUG_STREAM(CYAN("unwrap(") << WHITE(a) << CYAN(") -> ") << WHITE(res));
  return res;
};
template <typename T, typename T2> const T wrap(T2 a) {
  T res = static_cast<T>(a / K);
  ROS_DEBUG_STREAM(CYAN("convert wrap(")
                   << WHITE(a) << CYAN(") -> ") << WHITE(res));
  return res;
};
template <typename T, typename T2> const T unwrap(T2 a) {
  T res = static_cast<T>(a * K);
  ROS_DEBUG_STREAM(CYAN("convert unwrap(")
                   << WHITE(a) << CYAN(") -> ") << WHITE(res));
  return res;
};

} // namespace r2d2_process

#endif // R2D2_MATH_HPP
