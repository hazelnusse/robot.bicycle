#ifndef FILTER_H
#define FILTER_H

#include <cmath>

namespace control {

template <typename T>
constexpr int signum(T val)
{
  return (T(0) < val) - (val < T(0));
}

/** \brief Balanced realization of a Tustin approximation of a first order
 *         continuous filter
 */
template <typename Real>
class first_order_discrete_filter {
 public:
  /** \brief Constructor
   *
   * \description Continuous time filter is assumed to have form:
   *
   *    H(s) = (n1 * s + n0) / (s + d0)
   *
   * \param[in] n0 Zero order numerator coefficient.
   * \param[in] n1 First order numerator coefficient.
   * \param[in] d0 Zero order denominator coefficient.
   * \param[in] T  Sample time.
   *
   */
  first_order_discrete_filter(Real n0, Real n1, Real d0, Real T) :
    a{-(T*d0 - 2)/(T*d0 + 2)},
    c{signum(4*T*(-d0*n1 + n0)/(std::pow(T, Real(2))*std::pow(d0, Real(2)) + 4*T*d0 + 4))
      * std::sqrt(std::fabs(4*T*(-d0*n1 + n0)/(std::pow(T, Real(2))*std::pow(d0, Real(2)) + 4*T*d0 + 4)))},
    b{std::fabs(c)},
    d{(T*n0 + 2*n1)/(T*d0 + 2)},
    x{0.0f} {}
  const Real a, c, b, d;
  Real x;

  void update(Real u)
  {
    x = a*x + b*u;
  }

  Real output(Real u)
  {
    return c*x + d*u;
  }

  Real state()
  {
    return x;
  }

};

}

#endif
