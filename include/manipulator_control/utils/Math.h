#ifndef MATH_H_
#define MATH_H_

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace math {

template <typename T> constexpr const T &deg2rad(const T a) {
  return a * M_PI / 180;
};
template <typename T> constexpr const T &abs(const T a) { return std::abs(a); };
template <typename T> constexpr const T &min(const T a, const T b) {
  return std::min<T>(a, b);
};
template <typename T> constexpr const T &max(const T a, const T b) {
  return std::max<T>(a, b);
};
template <typename T> constexpr const T &sin(const T thetha) {
  return std::sin(deg2rad(thetha));
};

} // namespace math
#endif // MATH_H_
