#include <cstddef>
#include <vector>

double h_polynome(const std::vector<double> &coeff, double x) {
  if (coeff.empty())
    return 0;

  double result = coeff.back();
  for (size_t i = coeff.size() - 1; i > 0; --i)
    result = result * x + coeff[i - 1];
  return result;
}
