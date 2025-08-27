#ifndef MATH_H_
#define MATH_H_

#include <algorithm>
#include <cmath>
#include <cstdlib>

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

constexpr int K = -100;
template <typename T> constexpr T wrap(T a) { return a /= K; };
template <typename T> constexpr T unwrap(T a) { return a *= K; };

template <typename T, typename T2> constexpr T wrap(T2 a) {
  return static_cast<T>(a /= K);
};
template <typename T, typename T2> constexpr T unwrap(T2 a) {
  return static_cast<T>(a *= K);
};

} // namespace r2d2_process

#endif // MATH_H_
