#pragma once

#include <Eigen/Dense>

namespace docking
{

struct QuinticCoefficients
{
  double a0{0.0};
  double a1{0.0};
  double a2{0.0};
  double a3{0.0};
  double a4{0.0};
  double a5{0.0};
};

inline QuinticCoefficients fitQuintic(
  double p0, double v0, double a0,
  double p1, double v1, double a1)
{
  Eigen::Matrix<double, 6, 6> A;
  A << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 2.0, 0.0, 0.0, 0.0,
       1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
       0.0, 1.0, 2.0, 3.0, 4.0, 5.0,
       0.0, 0.0, 2.0, 6.0, 12.0, 20.0;
  Eigen::Matrix<double, 6, 1> b;
  b << p0, v0, a0, p1, v1, a1;
  const Eigen::Matrix<double, 6, 1> c = A.colPivHouseholderQr().solve(b);
  return QuinticCoefficients{c(0), c(1), c(2), c(3), c(4), c(5)};
}

inline double evalQuintic(const QuinticCoefficients & c, double u, int order = 0)
{
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  if (order == 0) {
    return c.a0 + c.a1 * u + c.a2 * u2 + c.a3 * u3 + c.a4 * u4 + c.a5 * u5;
  }
  if (order == 1) {
    return c.a1 + 2.0 * c.a2 * u + 3.0 * c.a3 * u2 + 4.0 * c.a4 * u3 + 5.0 * c.a5 * u4;
  }
  if (order == 2) {
    return 2.0 * c.a2 + 6.0 * c.a3 * u + 12.0 * c.a4 * u2 + 20.0 * c.a5 * u3;
  }
  return 0.0;
}

}  // 命名空间 docking
