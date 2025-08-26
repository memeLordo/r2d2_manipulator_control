#ifndef POLYNOME_H
#define POLYNOME_H

#include <cstddef>
// #include <vector>

namespace horner {

template <typename T, typename T2, size_t N>
constexpr T polynome(const T (&coeffs)[N], T2 x) {

  if (N == 0)
    return T{0};

  T x_ = static_cast<T>(x);
  T result = coeffs[N - 1];
  for (size_t i = N - 1; i > 0; --i)
    result = result * x_ + coeffs[i - 1];
  return result;
}

// template <typename T, typename T2>
// T polynome(const std::vector<T> &coeffs, T2 x) {
//   if (coeffs.empty())
//     return T{0};
//   T _x = static_cast<T>(x);
//   T result = coeffs.back();
//   for (size_t i = coeffs.size() - 1; i > 0; --i)
//     result = result * _x + coeffs[i - 1];
//   return result;
// }

} // namespace horner
#endif // POLYNOME_SAFE_H
