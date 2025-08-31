#ifndef POLYNOME_H
#define POLYNOME_H

// #include <cstddef>
#include <vector>

namespace horner {

template <typename T, typename T2>
T polynome(const std::vector<T> &coeffs, T2 x) {
  if (coeffs.empty())
    return T{0};
  T _x = static_cast<T>(x);
  T result = coeffs.back();
  for (size_t i = coeffs.size() - 1; i > 0; --i)
    result = result * _x + coeffs[i - 1];
  return result;
}

} // namespace horner
#endif // POLYNOME_SAFE_H
