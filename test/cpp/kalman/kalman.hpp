#ifndef KALMAN_TEST_HPP
#define KALMAN_TEST_HPP

#include "check.hpp"
#include "kalman/matrix.hpp"

#include <stdexcept>

void
matrix_test ()
{
  const matrix<3, 3> a{ 4, 1, 2, //
                        1, 3, 0, //
                        2, 0, 5 };
  const auto product = a * inverse (a);
  for (std::size_t r = 0; r < 3; ++r)
    {
      for (std::size_t c = 0; c < 3; ++c)
        {
          check_near (product (r, c), r == c ? 1.0 : 0.0, 1e-12,
                      "matrix inverse");
        }
    }
  check_throws<std::domain_error> (
      [] { (void)inverse (matrix<2, 2>{ 1, 2, 2, 4 }); }, "singular matrix");
}

#endif // !KALMAN_TEST_HPP
