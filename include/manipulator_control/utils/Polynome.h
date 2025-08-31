#ifndef POLYNOME_H
#define POLYNOME_H

#include <array>
#include <cstddef>

namespace horner {

template <typename T, size_t N, typename T2>
constexpr T polynome(const std::array<T, N> &coeffs, T2 x) {

  if (N == 0)
    return T{0};

  T x_ = static_cast<T>(x);
  T result = coeffs[N - 1];
  for (size_t i = N - 1; i > 0; --i)
    result = result * x_ + coeffs[i - 1];
  return result;
}

} // namespace horner
#endif // POLYNOME_SAFE_H
