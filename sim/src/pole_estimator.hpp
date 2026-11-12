#ifndef POLE_ESTIMATOR_HPP
#define POLE_ESTIMATOR_HPP

#include "kalman/kalman_filter.hpp"

#include <array>
#include <cmath>
#include <optional>

using vec3 = std::array<double, 3>;

inline double
dot (const vec3 &a, const vec3 &b)
{
  return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]);
}

inline vec3
cross (const vec3 &a, const vec3 &b)
{
  return { (a[1] * b[2]) - (a[2] * b[1]), (a[2] * b[0]) - (a[0] * b[2]),
           (a[0] * b[1]) - (a[1] * b[0]) };
}

inline vec3
minus (const vec3 &a, const vec3 &b)
{
  return { a[0] - b[0], a[1] - b[1], a[2] - b[2] };
}

inline double
norm (const vec3 &a)
{
  return std::sqrt (dot (a, a));
}

inline vec3
normalized (const vec3 &a)
{
  const double n = norm (a);
  return { a[0] / n, a[1] / n, a[2] / n };
}

// Where the pole hardware sits. Keep in sync with sim/worlds/pole_tracking.sdf.
// The UWB anchor and the camera are at the same point.
struct pole_camera
{
  vec3 position{ 24.0, 0.0, 3.0 };
  // The rest pose. On the pan-tilt mount these change with every frame.
  double yaw_rad{ 3.14159265 };
  double pitch_rad{ -0.1745 }; // negative tilts the view up
  double focal_px{ 960.0 };    // (width / 2) / tan (hfov / 2)
  double center_u_px{ 960.0 };
  double center_v_px{ 540.0 };

  // Unit vector in the world towards a pixel (u right, v down).
  [[nodiscard]] vec3
  ray (double u, double v) const
  {
    // Camera frame: x forward, y left, z up.
    const vec3 c{ 1.0, -(u - center_u_px) / focal_px,
                  -(v - center_v_px) / focal_px };
    const double cp = std::cos (pitch_rad);
    const double sp = std::sin (pitch_rad);
    const double cy = std::cos (yaw_rad);
    const double sy = std::sin (yaw_rad);
    const vec3 pitched{ (cp * c[0]) + (sp * c[2]), c[1],
                        (-sp * c[0]) + (cp * c[2]) };
    return normalized ({ (cy * pitched[0]) - (sy * pitched[1]),
                         (sy * pitched[0]) + (cy * pitched[1]), pitched[2] });
  }

  // The pixel a world point lands on, the inverse of ray (). Empty when the
  // point is behind the camera; it can still be outside the image.
  [[nodiscard]] std::optional<std::array<double, 2>>
  pixel (const vec3 &point) const
  {
    const vec3 d = minus (point, position);
    const double cp = std::cos (pitch_rad);
    const double sp = std::sin (pitch_rad);
    const double cy = std::cos (yaw_rad);
    const double sy = std::sin (yaw_rad);
    const double px = (cy * d[0]) + (sy * d[1]);
    const double py = (-sy * d[0]) + (cy * d[1]);
    const double forward = (cp * px) - (sp * d[2]);
    const double up = (sp * px) + (cp * d[2]);
    if (forward <= 1e-9)
      {
        return std::nullopt;
      }
    return std::array<double, 2>{ center_u_px - (focal_px * py / forward),
                                  center_v_px - (focal_px * up / forward) };
  }

  [[nodiscard]] bool
  sees (const vec3 &point) const
  {
    const auto p = pixel (point);
    return p && (*p)[0] >= 0 && (*p)[0] <= 2 * center_u_px && (*p)[1] >= 0
           && (*p)[1] <= 2 * center_v_px;
  }
};

struct pole_estimator_config
{
  double imu_accel_sigma_mps2{ 0.3 };
  double maneuver_accel_sigma_mps2{ 2.0 };
  double range_sigma_m{ 0.1 };
  double bearing_sigma_rad{ 4.0 / 960.0 }; // about 4 pixels
  double initial_velocity_sigma_mps{ 5.0 };
  // Chi-square gates at 99.9 % for 1, 2 and 3 dof.
  double gate_1d{ 10.83 };
  double gate_2d{ 13.82 };
  double gate_3d{ 16.27 };
};

// Fuses IMU acceleration, UWB range to the pole, camera bearing from the pole
// and GPS in a local ENU frame. State: [x, y, z, v_x, v_y, v_z]. Any subset of
// the sensors works. Range and bearing are nonlinear in the position, so they
// are linearized around the current estimate.
class pole_estimator
{
  pole_estimator_config config_;
  vec3 anchor_;
  kalman_filter<6> filter_{};
  bool initialized_{ false };
  double t_{ 0 };
  std::optional<vec3> accel_{};

  void
  propagate (double dt)
  {
    auto F = matrix<6, 6>::identity ();
    const double sigma = accel_ ? config_.imu_accel_sigma_mps2
                                : config_.maneuver_accel_sigma_mps2;
    const double q = sigma * sigma;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    matrix<6, 6> Q{};
    vector<6> u{};
    for (std::size_t i = 0; i < 3; ++i)
      {
        F (i, i + 3) = dt;
        Q (i, i) = q * dt3 / 3;
        Q (i, i + 3) = Q (i + 3, i) = q * dt2 / 2;
        Q (i + 3, i + 3) = q * dt;
        if (accel_)
          {
            u (i, 0) = (*accel_)[i] * dt2 / 2;
            u (i + 3, 0) = (*accel_)[i] * dt;
          }
      }
    filter_.predict (F, Q, u);
  }

public:
  pole_estimator (const pole_estimator_config &config, const vec3 &anchor)
      : config_ (config), anchor_ (anchor)
  {
  }

  [[nodiscard]] bool
  initialized () const noexcept
  {
    return initialized_;
  }

  [[nodiscard]] vec3
  position () const
  {
    const auto &x = filter_.state ();
    return { x (0, 0), x (1, 0), x (2, 0) };
  }

  [[nodiscard]] vec3
  velocity () const
  {
    const auto &x = filter_.state ();
    return { x (3, 0), x (4, 0), x (5, 0) };
  }

  // The time the state is valid for.
  [[nodiscard]] double
  time () const noexcept
  {
    return t_;
  }

  [[nodiscard]] double
  sigma_position (std::size_t axis) const
  {
    return std::sqrt (filter_.covariance () (axis, axis));
  }

  void
  initialize (double t, const vec3 &position, double sigma_m)
  {
    vector<6> x0{ position[0], position[1], position[2], 0, 0, 0 };
    matrix<6, 6> P0{};
    for (std::size_t i = 0; i < 3; ++i)
      {
        P0 (i, i) = sigma_m * sigma_m;
        P0 (i + 3, i + 3) = config_.initial_velocity_sigma_mps
                            * config_.initial_velocity_sigma_mps;
      }
    filter_ = kalman_filter<6>{ x0, P0 };
    t_ = t;
    initialized_ = true;
  }

  // Predicts forward to time t. Out of order times are ignored.
  void
  advance_to (double t)
  {
    if (initialized_ && t > t_)
      {
        propagate (t - t_);
        t_ = t;
      }
  }

  // Acceleration in the local frame, used from now until the next sample.
  void
  set_acceleration (const vec3 &accel)
  {
    accel_ = accel;
  }

  // The update functions return false if the measurement was gated out.
  bool
  add_range (double range_m)
  {
    const vec3 to_tag = minus (position (), anchor_);
    const double predicted = norm (to_tag);
    if (predicted < 1e-3)
      {
        return false;
      }
    const vec3 u = normalized (to_tag);
    const matrix<1, 6> H{ u[0], u[1], u[2], 0, 0, 0 };
    const vector<1> z{ range_m + dot (u, anchor_) };
    const matrix<1, 1> R{ config_.range_sigma_m * config_.range_sigma_m };
    if (filter_.nis (z, H, R) > config_.gate_1d)
      {
        return false;
      }
    filter_.update (z, H, R);
    return true;
  }

  bool
  add_bearing (const vec3 &ray)
  {
    // Measures the sideways offset of the state from the ray, in the plane
    // across it. Sideways uncertainty grows with distance.
    const vec3 t1 = normalized (cross (ray, { 0, 0, 1 }));
    const vec3 t2 = cross (ray, t1);
    const matrix<2, 6> H{ t1[0], t1[1], t1[2], 0, 0, 0, //
                          t2[0], t2[1], t2[2], 0, 0, 0 };
    const vector<2> z{ dot (t1, anchor_), dot (t2, anchor_) };
    const double sigma
        = config_.bearing_sigma_rad * norm (minus (position (), anchor_));
    const matrix<2, 2> R{ sigma * sigma, 0, 0, sigma * sigma };
    if (filter_.nis (z, H, R) > config_.gate_2d)
      {
        return false;
      }
    filter_.update (z, H, R);
    return true;
  }

  bool
  add_gps (const vec3 &position, double sigma_m, double sigma_up_m)
  {
    const matrix<3, 6> H{ 1, 0, 0, 0, 0, 0, //
                          0, 1, 0, 0, 0, 0, //
                          0, 0, 1, 0, 0, 0 };
    const vector<3> z{ position[0], position[1], position[2] };
    const matrix<3, 3> R{ sigma_m * sigma_m, 0, 0, 0,
                          sigma_m * sigma_m, 0, 0, 0,
                          sigma_up_m * sigma_up_m };
    if (filter_.nis (z, H, R) > config_.gate_3d)
      {
        return false;
      }
    filter_.update (z, H, R);
    return true;
  }

  // Horizontal velocity, what a GPS receiver reports.
  bool
  add_gps_velocity (double v_east, double v_north, double sigma_mps)
  {
    const matrix<2, 6> H{ 0, 0, 0, 1, 0, 0, //
                          0, 0, 0, 0, 1, 0 };
    const vector<2> z{ v_east, v_north };
    const matrix<2, 2> R{ sigma_mps * sigma_mps, 0, 0, sigma_mps * sigma_mps };
    if (filter_.nis (z, H, R) > config_.gate_2d)
      {
        return false;
      }
    filter_.update (z, H, R);
    return true;
  }
};

#endif // !POLE_ESTIMATOR_HPP
