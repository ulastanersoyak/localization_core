#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Core>
#include <Eigen/LU>

// Linear Kalman filter with N states; the linear algebra is Eigen's.
template <int N> class kalman_filter
{
public:
  using vector = Eigen::Matrix<double, N, 1>;
  using matrix = Eigen::Matrix<double, N, N>;

  kalman_filter () = default;
  kalman_filter (const vector &x0, const matrix &P0) : x_ (x0), P_ (P0) {}

  [[nodiscard]] const vector &
  state () const noexcept
  {
    return x_;
  }

  [[nodiscard]] const matrix &
  covariance () const noexcept
  {
    return P_;
  }

  // x = F x + u, P = F P F' + Q. u is the (already integrated) effect of any
  // known control input such as measured acceleration.
  void
  predict (const matrix &F, const matrix &Q, const vector &u = vector::Zero ())
  {
    x_ = F * x_ + u;
    P_ = F * P_ * F.transpose () + Q;
  }

  // Normalised innovation squared y' S^-1 y of measurement z. Follows a
  // chi-square distribution with M degrees of freedom when the filter is
  // consistent, so it can be used to gate outliers before update().
  template <int M>
  [[nodiscard]] double
  nis (const Eigen::Matrix<double, M, 1> &z,
       const Eigen::Matrix<double, M, N> &H,
       const Eigen::Matrix<double, M, M> &R) const
  {
    const Eigen::Matrix<double, M, 1> y = z - H * x_;
    const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose () + R;
    return y.dot (S.inverse () * y);
  }

  template <int M>
  void
  update (const Eigen::Matrix<double, M, 1> &z,
          const Eigen::Matrix<double, M, N> &H,
          const Eigen::Matrix<double, M, M> &R)
  {
    const Eigen::Matrix<double, M, 1> y = z - H * x_;
    const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose () + R;
    // S is at most 3 x 3 here, Eigen inverts that in closed form.
    const Eigen::Matrix<double, N, M> K = P_ * H.transpose () * S.inverse ();
    x_ += K * y;

    // Joseph form keeps P symmetric and positive definite.
    const matrix I_KH = matrix::Identity () - K * H;
    P_ = I_KH * P_ * I_KH.transpose () + K * R * K.transpose ();
  }

private:
  vector x_ = vector::Zero ();
  matrix P_ = matrix::Zero ();
};

#endif // !KALMAN_FILTER_HPP
