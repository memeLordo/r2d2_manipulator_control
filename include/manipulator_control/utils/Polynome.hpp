#ifndef R2D2_POLYNOME_HPP
#define R2D2_POLYNOME_HPP

#include <vector>

namespace horner {
template <typename T>
constexpr T polynome(const std::vector<T>& coeffs, const T x) {
  if (coeffs.empty()) return T{};

  T result_{coeffs[0]};
  for (size_t i = 1; i < coeffs.size(); ++i) result_ = result_ * x + coeffs[i];
  return result_;
};
}  // namespace horner
#endif  // R2D2_POLYNOME_H
