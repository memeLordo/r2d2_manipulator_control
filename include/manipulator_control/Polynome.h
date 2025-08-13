#ifndef POLYNOME_H
#define POLYNOME_H

#include <cstddef>
#include <vector>

namespace Horner {

template <typename T, size_t N>
constexpr T polynome(const T (&coeffs)[N], T x) {
  // static_assert(N > 0, "Coefficient array cannot be empty");
  if (N == 0)
    return T{0};
  T result = coeffs[N - 1];
  for (size_t i = N - 1; i > 0; --i)
    result = result * x + coeffs[i - 1];
  return result;
}

template <typename T> T polynome(const std::vector<T> &coeffs, T x) {
  if (coeffs.empty())
    return T{0};

  T result = coeffs.back();
  for (size_t i = coeffs.size() - 1; i > 0; --i)
    result = result * x + coeffs[i - 1];
  return result;
}

} // namespace Horner
#endif // POLYNOME_SAFE_H
