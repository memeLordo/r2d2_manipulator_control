#ifndef POLYNOME_H
#define POLYNOME_H

// #include <cstddef>
#include <vector>

namespace horner {

template <typename T>
constexpr T polynome(const std::vector<T> &coeffs, const T x) {
  if (coeffs.empty())
    return T{0};
  T result_ = coeffs[0];
  for (size_t i = 1; i < coeffs.size(); ++i)
    result_ = result_ * x + coeffs[i];
  return result_;
}

} // namespace horner
#endif // POLYNOME_SAFE_H
