// Tracks the drone from a pole beside its flight path and compares the result
// with the ground truth:
//
//   /pole/detection (from sim/detector/detect.py) -> camera bearing
//   simulated UWB anchor on the pole, 20 Hz       -> range
//   /drone/imu (100 Hz), /drone/navsat (1 Hz)     -> optional, see --no-imu,
//                                                    --no-gps
//   /model/drone/pose                             -> ground truth, used to
//                                                    score and to fly only
//   /model/pole_camera/pose                       -> where the pan-tilt
//                                                    camera pointed
//
// The camera sits on a pan-tilt mount and is turned towards the estimate of
// the drone (never the ground truth), slew rate limited, so it keeps the drone
// in view. Each detection is turned into a ray with the camera pose at the
// moment the frame was taken.
//
// Detections arrive late because YOLO takes time. All measurements go through
// a buffer and are fused in the order of their timestamps, a fixed time behind
// the simulation, so each is applied at the moment it was measured.

#include "error_stats.hpp"
#include "local_frame.hpp"
#include "pole_estimator.hpp"

#include "mti_630/data_packet_parser.hpp"
#include "utility/queue.hpp"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/transport/Node.hh>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <numbers>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <variant>

namespace
{
volatile std::sig_atomic_t stop_requested = 0;

void
handle_signal (int /*signal*/)
{
  stop_requested = 1;
}

template <typename... Ts> struct overloaded : Ts...
{
  using Ts::operator()...;
};

constexpr double pi = std::numbers::pi;
// Keep in sync with <gravity> in sim/worlds/pole_tracking.sdf.
constexpr double world_gravity = 9.8;

struct options
{
  bool use_gps{ true };
  bool use_imu{ true };
  double uwb_noise_m{ 0.1 };
  double gps_noise_m{ 1.5 };
  double gps_vertical_factor{ 1.7 };
  double speed_noise_mps{ 0.05 };
  double accel_bias_mps2{ 0.02 };
  double tilt_error_deg{ 0.2 };
  double heading_error_deg{ 1.0 };
  double orientation_error_tau_s{ 30.0 };
  double latency_s{ 0.4 };
  double duration_s{ 0.0 };
  double radius_m{ 8.0 };
  double altitude_m{ 8.0 };
  double altitude_swing_m{ 3.0 };
  double origin_lat{ 39.97656 };
  double origin_lon{ 32.73812 };
  double origin_alt{ 894.0 };
  unsigned seed{ 1 };
  bool follow{ true };
  bool track_camera{ true };
  double slew_rate_dps{ 120.0 };
  std::string csv_path{};
};

void
print_usage (const char *name)
{
  std::cout
      << "usage: " << name << " [options]\n"
      << "  --no-gps           do not use the GPS\n"
      << "  --no-imu           do not use the IMU\n"
      << "  --uwb-noise M      UWB range noise, metres (0.1)\n"
      << "  --gps-noise M      white GPS position noise, metres (1.5)\n"
      << "  --latency S        fuse measurements this far behind the "
         "simulation (0.4)\n"
      << "  --duration S       stop after S simulated seconds (run forever)\n"
      << "  --radius M         figure-eight loop radius (8)\n"
      << "  --altitude M       cruise height above the ground (8)\n"
      << "  --altitude-swing M climb and descend this far around it (3)\n"
      << "  --seed N           noise seed (1)\n"
      << "  --csv FILE         write estimate and ground truth to FILE\n"
      << "  --no-follow        do not make the GUI camera follow the drone\n"
      << "  --fixed-camera     keep the pole camera in its rest pose instead "
         "of\n"
      << "                     turning it towards the drone\n"
      << "  --slew-rate D      pan-tilt speed limit, degrees per second (120)\n";
}

std::optional<options>
parse_options (int argc, char *argv[])
{
  options opt;
  for (int i = 1; i < argc; ++i)
    {
      const std::string arg = argv[i];
      const auto text = [&] () -> std::string
        {
          if (i + 1 >= argc)
            {
              throw std::invalid_argument ("missing value for " + arg);
            }
          return argv[++i];
        };
      const auto value = [&] { return std::stod (text ()); };
      if (arg == "--no-gps")
        opt.use_gps = false;
      else if (arg == "--no-imu")
        opt.use_imu = false;
      else if (arg == "--uwb-noise")
        opt.uwb_noise_m = value ();
      else if (arg == "--gps-noise")
        opt.gps_noise_m = value ();
      else if (arg == "--latency")
        opt.latency_s = value ();
      else if (arg == "--duration")
        opt.duration_s = value ();
      else if (arg == "--radius")
        opt.radius_m = value ();
      else if (arg == "--altitude")
        opt.altitude_m = value ();
      else if (arg == "--altitude-swing")
        opt.altitude_swing_m = value ();
      else if (arg == "--seed")
        opt.seed = static_cast<unsigned> (value ());
      else if (arg == "--csv")
        opt.csv_path = text ();
      else if (arg == "--no-follow")
        opt.follow = false;
      else if (arg == "--fixed-camera")
        opt.track_camera = false;
      else if (arg == "--slew-rate")
        opt.slew_rate_dps = value ();
      else
        return std::nullopt;
    }
  return opt;
}

double
stamp_seconds (const gz::msgs::Header &header)
{
  return static_cast<double> (header.stamp ().sec ())
         + (header.stamp ().nsec () * 1e-9);
}

double
wrap_angle (double a)
{
  while (a > pi)
    a -= 2 * pi;
  while (a < -pi)
    a += 2 * pi;
  return a;
}

using quaternion = std::array<double, 4>; // w, x, y, z

quaternion
multiply (const quaternion &a, const quaternion &b)
{
  const auto &[w1, x1, y1, z1] = a;
  const auto &[w2, x2, y2, z2] = b;
  return { (w1 * w2) - (x1 * x2) - (y1 * y2) - (z1 * z2),
           (w1 * x2) + (x1 * w2) + (y1 * z2) - (z1 * y2),
           (w1 * y2) - (x1 * z2) + (y1 * w2) + (z1 * x2),
           (w1 * z2) + (x1 * y2) - (y1 * x2) + (z1 * w2) };
}

quaternion
from_rotation_vector (const vec3 &r)
{
  const double angle = norm (r);
  if (angle < 1e-12)
    {
      return { 1, 0, 0, 0 };
    }
  const double s = std::sin (angle / 2) / angle;
  return { std::cos (angle / 2), r[0] * s, r[1] * s, r[2] * s };
}

struct imu_event
{
  Xbus sample;
};

struct navsat_event
{
  double lat, lon, alt, v_east, v_north;
};

struct range_event
{
  double range_m;
};

struct detection_event
{
  double u, v, confidence;
};

using measurement
    = std::variant<imu_event, navsat_event, range_event, detection_event>;

struct timed_measurement
{
  double t;
  measurement value;
};

struct truth_sample
{
  double t;
  vec3 position;
  double yaw;
};

// Where the pan-tilt camera pointed.
struct camera_sample
{
  double t;
  double yaw;
  double pitch;
};

using inbound = std::variant<timed_measurement, truth_sample, camera_sample>;

// Distance error split into its horizontal and vertical parts.
struct scored_error
{
  error_stats total{};
  error_stats horizontal{};
  error_stats vertical{};

  void
  add (const vec3 &estimate, const vec3 &truth)
  {
    const vec3 d = minus (estimate, truth);
    total.add (norm (d));
    horizontal.add (std::hypot (d[0], d[1]));
    vertical.add (std::abs (d[2]));
  }
};

class pole_tracking_sim
{
  options opt_;
  pole_camera camera_{};
  gz::transport::Node node_{};
  gz::transport::Node::Publisher cmd_pub_{};
  gz::transport::Node::Publisher overlay_pub_{};
  queue<inbound> inbox_{};
  std::multimap<double, measurement> buffer_{};
  local_frame frame_;
  pole_estimator estimator_;
  std::mt19937 rng_;
  std::normal_distribution<double> noise_{ 0.0, 1.0 };
  vec3 accel_bias_{};
  vec3 orientation_error_{};
  double last_imu_t_{ -1 };

  std::deque<truth_sample> truth_{};
  std::deque<camera_sample> camera_history_{};
  double command_yaw_{ pole_camera{}.yaw_rad };
  double command_pitch_{ pole_camera{}.pitch_rad };
  double last_steer_t_{ -1 };
  std::size_t frames_scored_{ 0 };
  std::size_t frames_in_view_{ 0 };
  double sim_t_{ 0 };
  double last_range_t_{ -1 };
  double last_cmd_t_{ -1 };
  double last_follow_t_{ -100 };
  double last_score_t_{ -1 };
  double last_print_t_{ 0 };
  double last_overlay_t_{ -1 };
  double last_draw_t_{ -1 };

  double turn_direction_{ 1 };
  double turned_{ 0 };
  bool airborne_{ false };
  std::optional<double> last_yaw_{};

  std::optional<double> latest_range_{};
  std::optional<vec3> latest_raw_fix_{};
  std::size_t rejected_in_a_row_{ 0 };
  static constexpr std::size_t max_rejected_detections = 10;

  scored_error fused_error_{};
  scored_error raw_fix_error_{};
  scored_error gps_error_{};
  std::size_t ranges_{ 0 };
  std::size_t ranges_rejected_{ 0 };
  std::size_t detections_{ 0 };
  std::size_t detections_rejected_{ 0 };
  std::size_t detections_late_{ 0 };
  std::size_t gps_fixes_{ 0 };
  std::size_t gps_rejected_{ 0 };
  std::size_t resets_{ 0 };
  std::size_t diverged_{ 0 };
  std::ofstream csv_{};

  static pole_estimator_config
  make_config (const options &opt)
  {
    pole_estimator_config config{};
    config.range_sigma_m = std::max (opt.uwb_noise_m, 0.02);
    return config;
  }

public:
  explicit pole_tracking_sim (const options &opt)
      : opt_ (opt), frame_ (opt.origin_lat, opt.origin_lon, opt.origin_alt),
        estimator_ (make_config (opt), pole_camera{}.position), rng_ (opt.seed)
  {
    for (auto &b : accel_bias_)
      {
        b = opt_.accel_bias_mps2 * noise_ (rng_);
      }
    if (!opt_.csv_path.empty ())
      {
        csv_.open (opt_.csv_path);
        csv_ << "t,truth_x,truth_y,truth_z,est_x,est_y,est_z,error_3d\n";
      }
  }

  bool
  start ()
  {
    cmd_pub_ = node_.Advertise<gz::msgs::Twist> ("/drone/cmd_vel");
    overlay_pub_
        = node_.Advertise<gz::msgs::StringMsg> ("/positioning/overlay");
    bool ok = node_.Subscribe ("/model/drone/pose",
                               &pole_tracking_sim::on_pose, this)
              && node_.Subscribe ("/pole/detection",
                                  &pole_tracking_sim::on_detection, this)
              && node_.Subscribe ("/model/pole_camera/pose",
                                  &pole_tracking_sim::on_camera_pose, this);
    if (opt_.use_imu)
      {
        ok = ok
             && node_.Subscribe ("/drone/imu", &pole_tracking_sim::on_imu,
                                 this);
      }
    if (opt_.use_gps)
      {
        ok = ok
             && node_.Subscribe ("/drone/navsat",
                                 &pole_tracking_sim::on_navsat, this);
      }
    if (!ok)
      {
        std::cerr << "failed to subscribe to the gazebo topics\n";
      }
    return ok;
  }

  void
  run ()
  {
    std::cout << "waiting for the simulation (is `gz sim` running?)...\n"
              << "sensors: uwb, camera" << (opt_.use_imu ? ", imu" : "")
              << (opt_.use_gps ? ", gps" : "") << std::endl;
    while (stop_requested == 0)
      {
        auto event = inbox_.pop_for (std::chrono::milliseconds (200));
        if (!event)
          {
            continue;
          }
        std::visit (overloaded{
                        [this] (timed_measurement &m) { receive (m); },
                        [this] (const truth_sample &s) { on_truth (s); },
                        [this] (const camera_sample &s) { on_camera (s); },
                    },
                    *event);
        process_buffer ();

        if (sim_t_ - last_print_t_ >= 1.0)
          {
            last_print_t_ = sim_t_;
            print_status ();
          }
        if (sim_t_ - last_overlay_t_ >= 0.2)
          {
            last_overlay_t_ = sim_t_;
            publish_overlay ();
          }
        if (opt_.duration_s > 0 && sim_t_ >= opt_.duration_s)
          {
            break;
          }
      }
    cmd_pub_.Publish (gz::msgs::Twist{});
    print_summary ();
  }

private:
  // -- gz-transport callbacks (other threads): only convert and enqueue --

  void
  on_imu (const gz::msgs::IMU &msg)
  {
    Xbus sample{};
    const auto &q = msg.orientation ();
    sample.quat = { q.w (), q.x (), q.y (), q.z () };
    sample.quaternionAvailable = true;
    sample.quatFrame = xbus_frame::ENU;
    const auto &a = msg.linear_acceleration ();
    sample.acc = { a.x (), a.y (), a.z () };
    sample.accAvailable = true;
    inbox_.push (timed_measurement{ stamp_seconds (msg.header ()),
                                    imu_event{ sample } });
  }

  void
  on_navsat (const gz::msgs::NavSat &msg)
  {
    inbox_.push (timed_measurement{
        stamp_seconds (msg.header ()),
        navsat_event{ msg.latitude_deg (), msg.longitude_deg (),
                      msg.altitude (), msg.velocity_east (),
                      msg.velocity_north () } });
  }

  // "<frame time> <u> <v> <confidence>" from detect.py
  void
  on_detection (const gz::msgs::StringMsg &msg)
  {
    std::istringstream in (msg.data ());
    double t{ 0 };
    detection_event d{};
    if (in >> t >> d.u >> d.v >> d.confidence)
      {
        inbox_.push (timed_measurement{ t, d });
      }
  }

  void
  on_pose (const gz::msgs::Pose_V &msg)
  {
    for (const auto &pose : msg.pose ())
      {
        if (pose.name () != "drone")
          {
            continue;
          }
        const auto &q = pose.orientation ();
        const double yaw
            = std::atan2 (2 * (q.w () * q.z () + q.x () * q.y ()),
                          1 - 2 * (q.y () * q.y () + q.z () * q.z ()));
        const double t = pose.has_header () ? stamp_seconds (pose.header ())
                                            : stamp_seconds (msg.header ());
        const auto &p = pose.position ();
        inbox_.push (truth_sample{ t, { p.x (), p.y (), p.z () }, yaw });
        return;
      }
  }

  void
  on_camera_pose (const gz::msgs::Pose_V &msg)
  {
    for (const auto &pose : msg.pose ())
      {
        if (pose.name () != "pole_camera")
          {
            continue;
          }
        const auto &q = pose.orientation ();
        const double yaw
            = std::atan2 (2 * (q.w () * q.z () + q.x () * q.y ()),
                          1 - 2 * (q.y () * q.y () + q.z () * q.z ()));
        const double pitch = std::asin (
            std::clamp (2 * (q.w () * q.y () - q.z () * q.x ()), -1.0, 1.0));
        const double t = pose.has_header () ? stamp_seconds (pose.header ())
                                            : stamp_seconds (msg.header ());
        inbox_.push (camera_sample{ t, yaw, pitch });
        return;
      }
  }

  // -- main thread -----------------------------------------------------

  void
  on_camera (const camera_sample &s)
  {
    camera_history_.push_back (s);
    while (!camera_history_.empty ()
           && camera_history_.front ().t < s.t - opt_.latency_s - 3)
      {
        camera_history_.pop_front ();
      }
  }

  // The camera as it pointed at time t, interpolated between pose samples.
  [[nodiscard]] std::optional<pole_camera>
  camera_at (double t) const
  {
    if (camera_history_.empty () || t > camera_history_.back ().t + 0.05)
      {
        return std::nullopt;
      }
    pole_camera c = camera_;
    if (t <= camera_history_.front ().t)
      {
        c.yaw_rad = camera_history_.front ().yaw;
        c.pitch_rad = camera_history_.front ().pitch;
        return c;
      }
    for (std::size_t i = 1; i < camera_history_.size (); ++i)
      {
        const auto &b = camera_history_[i];
        if (b.t < t)
          {
            continue;
          }
        const auto &a = camera_history_[i - 1];
        const double span = b.t - a.t;
        const double w = span > 0 ? (t - a.t) / span : 0;
        c.yaw_rad = a.yaw + w * wrap_angle (b.yaw - a.yaw);
        c.pitch_rad = a.pitch + w * (b.pitch - a.pitch);
        return c;
      }
    c.yaw_rad = camera_history_.back ().yaw;
    c.pitch_rad = camera_history_.back ().pitch;
    return c;
  }

  // Turns the pan-tilt mount towards where the estimate says the drone is
  // now. The estimate lags the simulation by the fusion latency, so it is
  // carried forward with its velocity, plus a little lead for the mount.
  void
  steer ()
  {
    if (!opt_.track_camera || !estimator_.initialized ()
        || sim_t_ - last_steer_t_ < 0.02)
      {
        return;
      }
    const double dt = last_steer_t_ < 0 ? 0.02 : sim_t_ - last_steer_t_;
    last_steer_t_ = sim_t_;

    constexpr double lead_s = 0.1;
    const double ahead = sim_t_ - estimator_.time () + lead_s;
    const vec3 p = estimator_.position ();
    const vec3 v = estimator_.velocity ();
    const vec3 d = minus (
        { p[0] + ahead * v[0], p[1] + ahead * v[1], p[2] + ahead * v[2] },
        camera_.position);
    if (!std::isfinite (d[0]) || !std::isfinite (d[1])
        || !std::isfinite (d[2]))
      {
        return;
      }
    const double want_yaw = std::atan2 (d[1], d[0]);
    const double want_pitch = std::clamp (
        -std::atan2 (d[2], std::hypot (d[0], d[1])), -80 * pi / 180,
        30 * pi / 180);

    const double step = opt_.slew_rate_dps * pi / 180 * dt;
    command_yaw_ = wrap_angle (
        command_yaw_
        + std::clamp (wrap_angle (want_yaw - command_yaw_), -step, step));
    command_pitch_
        += std::clamp (want_pitch - command_pitch_, -step, step);

    // Rotation about z by yaw, then about the new y by pitch (roll 0).
    const double cy = std::cos (command_yaw_ / 2);
    const double sy = std::sin (command_yaw_ / 2);
    const double cp = std::cos (command_pitch_ / 2);
    const double sp = std::sin (command_pitch_ / 2);
    gz::msgs::Pose pose;
    pose.set_name ("pole_camera");
    pose.mutable_position ()->set_x (camera_.position[0]);
    pose.mutable_position ()->set_y (camera_.position[1]);
    pose.mutable_position ()->set_z (camera_.position[2]);
    pose.mutable_orientation ()->set_w (cy * cp);
    pose.mutable_orientation ()->set_x (-sy * sp);
    pose.mutable_orientation ()->set_y (cy * sp);
    pose.mutable_orientation ()->set_z (sy * cp);
    // set_pose replies, so a one-way request would never reach it.
    std::function<void (const gz::msgs::Boolean &, const bool)> ignore
        = [] (const gz::msgs::Boolean &, const bool) {};
    node_.Request ("/world/pole_tracking/set_pose", pose, ignore);
  }

  void
  receive (timed_measurement &m)
  {
    if (std::holds_alternative<detection_event> (m.value)
        && m.t < sim_t_ - opt_.latency_s)
      {
        ++detections_late_;
        return;
      }
    buffer_.emplace (m.t, std::move (m.value));
  }

  void
  on_truth (const truth_sample &s)
  {
    sim_t_ = std::max (sim_t_, s.t);
    truth_.push_back (s);
    while (!truth_.empty () && truth_.front ().t < s.t - opt_.latency_s - 3)
      {
        truth_.pop_front ();
      }

    // The UWB radio: the true distance plus noise, at 20 Hz.
    if (s.t - last_range_t_ >= 0.05)
      {
        last_range_t_ = s.t;
        const double range = norm (minus (s.position, camera_.position))
                             + (opt_.uwb_noise_m * noise_ (rng_));
        buffer_.emplace (s.t, range_event{ range });
      }

    fly (s);
    steer ();
    follow_drone ();
  }

  void
  process_buffer ()
  {
    while (!buffer_.empty ()
           && buffer_.begin ()->first <= sim_t_ - opt_.latency_s)
      {
        const auto node = buffer_.extract (buffer_.begin ());
        const double t = node.key ();
        estimator_.advance_to (t);
        std::visit ([this, t] (const auto &m) { handle (t, m); },
                    node.mapped ());
        if (estimator_.initialized () && !estimator_.healthy ())
          {
            // Start over from the next measurement rather than carry a
            // broken state (and steer the camera with it).
            estimator_.reset ();
            ++diverged_;
            std::cerr << "estimate diverged at t=" << t << ", restarting"
                      << std::endl;
          }
        if (estimator_.initialized ())
          {
            score (t);
          }
      }
  }

  void
  handle (double t, const imu_event &e)
  {
    const auto accel = with_imu_errors (t, e.sample)
                           .accelerationENU (world_gravity);
    if (accel)
      {
        estimator_.set_acceleration (*accel);
      }
  }

  // The same IMU error model as positioning_sim.
  [[nodiscard]] Xbus
  with_imu_errors (double t, Xbus sample)
  {
    const double deg = pi / 180;
    const vec3 sigma{ opt_.tilt_error_deg * deg, opt_.tilt_error_deg * deg,
                      opt_.heading_error_deg * deg };
    const double phi
        = last_imu_t_ < 0
              ? 0
              : std::exp (-(t - last_imu_t_) / opt_.orientation_error_tau_s);
    last_imu_t_ = t;
    for (std::size_t i = 0; i < 3; ++i)
      {
        orientation_error_[i] = (phi * orientation_error_[i])
                                + (sigma[i] * std::sqrt (1 - phi * phi)
                                   * noise_ (rng_));
        sample.acc[i] += accel_bias_[i];
      }
    sample.quat
        = multiply (from_rotation_vector (orientation_error_), sample.quat);
    return sample;
  }

  void
  handle (double t, const navsat_event &e)
  {
    const auto enu = frame_.to_enu (e.lat, e.lon, e.alt);
    const vec3 fix{ enu[0] + (opt_.gps_noise_m * noise_ (rng_)),
                    enu[1] + (opt_.gps_noise_m * noise_ (rng_)),
                    enu[2]
                        + (opt_.gps_vertical_factor * opt_.gps_noise_m
                           * noise_ (rng_)) };
    ++gps_fixes_;
    score_against_truth (t, fix, gps_error_);
    draw_gps_fix (fix);

    const double sigma = std::max (opt_.gps_noise_m, 1.0);
    if (!estimator_.initialized ())
      {
        estimator_.initialize (t, fix, 2 * sigma);
        return;
      }
    if (!estimator_.add_gps (fix, sigma, sigma * opt_.gps_vertical_factor))
      {
        ++gps_rejected_;
        return;
      }
    (void)estimator_.add_gps_velocity (
        e.v_east + (opt_.speed_noise_mps * noise_ (rng_)),
        e.v_north + (opt_.speed_noise_mps * noise_ (rng_)), 0.3);
  }

  void
  handle (double /*t*/, const range_event &e)
  {
    latest_range_ = e.range_m;
    if (!estimator_.initialized ())
      {
        return;
      }
    ++ranges_;
    if (!estimator_.add_range (e.range_m))
      {
        ++ranges_rejected_;
      }
  }

  void
  handle (double t, const detection_event &e)
  {
    ++detections_;
    if (!latest_range_)
      {
        return;
      }
    // Where the camera pointed when it took the frame.
    const auto camera = camera_at (t);
    if (!camera)
      {
        return;
      }
    // The direct answer: the point on the camera ray at the UWB distance.
    const vec3 ray = camera->ray (e.u, e.v);
    const vec3 raw{ camera_.position[0] + (*latest_range_ * ray[0]),
                    camera_.position[1] + (*latest_range_ * ray[1]),
                    camera_.position[2] + (*latest_range_ * ray[2]) };
    latest_raw_fix_ = raw;
    score_against_truth (t, raw, raw_fix_error_);

    if (!estimator_.initialized ())
      {
        estimator_.initialize (t, raw, 1.0);
        return;
      }
    if (estimator_.add_bearing (ray))
      {
        rejected_in_a_row_ = 0;
        return;
      }
    ++detections_rejected_;
    if (++rejected_in_a_row_ > max_rejected_detections)
      {
        // Many detections in a row disagree: trust them, not the state.
        estimator_.initialize (t, raw, 1.0);
        rejected_in_a_row_ = 0;
        ++resets_;
      }
  }

  [[nodiscard]] std::optional<vec3>
  truth_at (double t) const
  {
    if (truth_.size () < 2 || t < truth_.front ().t || t > truth_.back ().t)
      {
        return std::nullopt;
      }
    for (std::size_t i = 1; i < truth_.size (); ++i)
      {
        const auto &b = truth_[i];
        if (b.t < t)
          {
            continue;
          }
        const auto &a = truth_[i - 1];
        const double span = b.t - a.t;
        const double w = span > 0 ? (t - a.t) / span : 0;
        return vec3{ a.position[0] + w * (b.position[0] - a.position[0]),
                     a.position[1] + w * (b.position[1] - a.position[1]),
                     a.position[2] + w * (b.position[2] - a.position[2]) };
      }
    return std::nullopt;
  }

  void
  score_against_truth (double t, const vec3 &estimate, scored_error &stats)
  {
    if (const auto truth = truth_at (t))
      {
        stats.add (estimate, *truth);
      }
  }

  void
  score (double t)
  {
    if (t - last_score_t_ < 0.02)
      {
        return;
      }
    const auto truth = truth_at (t);
    if (!truth)
      {
        return;
      }
    last_score_t_ = t;
    if (const auto camera = camera_at (t))
      {
        ++frames_scored_;
        if (camera->sees (*truth))
          {
            ++frames_in_view_;
          }
      }
    const vec3 est = estimator_.position ();
    fused_error_.add (est, *truth);
    if (csv_.is_open ())
      {
        csv_ << std::fixed << std::setprecision (3) << t << ',' << (*truth)[0]
             << ',' << (*truth)[1] << ',' << (*truth)[2] << ',' << est[0]
             << ',' << est[1] << ',' << est[2] << ','
             << norm (minus (est, *truth)) << '\n';
      }
    if (t - last_draw_t_ >= 1.0 / 30.0)
      {
        last_draw_t_ = t;
        draw (est, *truth);
      }
  }

  // Takes off, then flies a figure eight with the speed and height changing.
  // The autopilot uses the true pose; the estimator never sees it.
  void
  fly (const truth_sample &s)
  {
    if (last_yaw_ && airborne_)
      {
        turned_ += wrap_angle (s.yaw - *last_yaw_);
      }
    last_yaw_ = s.yaw;
    if (s.t - last_cmd_t_ < 0.05)
      {
        return;
      }
    last_cmd_t_ = s.t;

    gz::msgs::Twist cmd;
    if (s.t > 3.0)
      {
        constexpr double swing_period_s = 17.0;
        const double swing_rate = 2 * pi / swing_period_s;
        const double target
            = opt_.altitude_m
              + opt_.altitude_swing_m * std::sin (swing_rate * s.t);
        const double feed_forward
            = airborne_
                  ? opt_.altitude_swing_m * swing_rate
                        * std::cos (swing_rate * s.t)
                  : 0.0;
        cmd.mutable_linear ()->set_z (
            std::clamp (feed_forward + target - s.position[2], -2.0, 2.0));

        airborne_ = airborne_ || s.position[2] > opt_.altitude_m / 2;
        if (airborne_)
          {
            const double speed = 2.5 + 1.5 * std::sin (2 * pi * s.t / 23.0);
            if (std::abs (turned_) >= 2 * pi)
              {
                turn_direction_ = -turn_direction_;
                turned_ = 0;
              }
            cmd.mutable_linear ()->set_x (speed);
            cmd.mutable_angular ()->set_z (turn_direction_ * speed
                                           / opt_.radius_m);
          }
      }
    cmd_pub_.Publish (cmd);
  }

  void
  follow_drone ()
  {
    if (!opt_.follow || sim_t_ > 30 || sim_t_ - last_follow_t_ < 3)
      {
        return;
      }
    last_follow_t_ = sim_t_;
    gz::msgs::Vector3d offset;
    offset.set_x (-6);
    offset.set_y (-3);
    offset.set_z (3.5);
    gz::msgs::StringMsg target;
    target.set_data ("drone");
    std::function<void (const gz::msgs::Boolean &, const bool)> ignore
        = [] (const gz::msgs::Boolean &, const bool) {};
    node_.Request ("/gui/follow/offset", offset, ignore);
    node_.Request ("/gui/follow", target, ignore);
  }

  // -- drawing -----------------------------------------------------------

  static gz::msgs::Marker
  marker (const std::string &ns, std::uint64_t id, gz::msgs::Marker::Type type,
          const vec3 &colour)
  {
    gz::msgs::Marker m;
    m.set_ns (ns);
    m.set_id (id);
    m.set_action (gz::msgs::Marker::ADD_MODIFY);
    m.set_type (type);
    m.set_visibility (gz::msgs::Marker::GUI);
    for (auto *c : { m.mutable_material ()->mutable_ambient (),
                     m.mutable_material ()->mutable_diffuse () })
      {
        c->set_r (static_cast<float> (colour[0]));
        c->set_g (static_cast<float> (colour[1]));
        c->set_b (static_cast<float> (colour[2]));
        c->set_a (0.9F);
      }
    return m;
  }

  static void
  add_point (gz::msgs::Marker &m, const vec3 &p)
  {
    auto *out = m.add_point ();
    out->set_x (p[0]);
    out->set_y (p[1]);
    out->set_z (p[2]);
  }

  void
  sphere (const std::string &ns, std::uint64_t id, const vec3 &position,
          double size, const vec3 &colour)
  {
    auto m = marker (ns, id, gz::msgs::Marker::SPHERE, colour);
    m.mutable_pose ()->mutable_position ()->set_x (position[0]);
    m.mutable_pose ()->mutable_position ()->set_y (position[1]);
    m.mutable_pose ()->mutable_position ()->set_z (position[2]);
    m.mutable_pose ()->mutable_orientation ()->set_w (1);
    m.mutable_scale ()->set_x (size);
    m.mutable_scale ()->set_y (size);
    m.mutable_scale ()->set_z (size);
    node_.Request ("/marker", m);
  }

  void
  draw (const vec3 &estimate, const vec3 &truth)
  {
    sphere ("estimate", 1, estimate, 0.3, { 0.1, 0.9, 0.2 });

    auto error_line = marker ("estimate", 2, gz::msgs::Marker::LINE_LIST,
                              { 1, 0.1, 0.1 });
    add_point (error_line, estimate);
    add_point (error_line, truth);
    node_.Request ("/marker", error_line);

    if (latest_raw_fix_)
      {
        sphere ("raw", 1, *latest_raw_fix_, 0.2, { 1, 0.5, 0 });
        auto ray = marker ("raw", 2, gz::msgs::Marker::LINE_LIST,
                           { 0.2, 0.5, 1 });
        add_point (ray, camera_.position);
        add_point (ray, *latest_raw_fix_);
        node_.Request ("/marker", ray);
      }

    // The UWB sphere sliced at the estimated height: the circle from one
    // anchor once the height is known.
    if (latest_range_)
      {
        const double h = estimate[2] - camera_.position[2];
        const double r2 = (*latest_range_ * *latest_range_) - (h * h);
        if (r2 > 0)
          {
            const double r = std::sqrt (r2);
            auto circle = marker ("uwb", 1, gz::msgs::Marker::LINE_STRIP,
                                  { 0.1, 0.8, 0.9 });
            for (int i = 0; i <= 96; ++i)
              {
                const double a = 2 * pi * i / 96;
                add_point (circle, { camera_.position[0] + r * std::cos (a),
                                     camera_.position[1] + r * std::sin (a),
                                     estimate[2] });
              }
            node_.Request ("/marker", circle);
          }
      }
  }

  void
  draw_gps_fix (const vec3 &fix)
  {
    sphere ("gps", gps_fixes_ % 20, { fix[0], fix[1], std::max (fix[2], 0.1) },
            0.2, { 0.9, 0.9, 0.1 });
  }

  // -- reporting ---------------------------------------------------------

  void
  print_status () const
  {
    std::cout << std::fixed << std::setprecision (2) << "t=" << std::setw (6)
              << sim_t_ << "s | fused error: last "
              << fused_error_.total.last () << " m, mean "
              << fused_error_.total.mean () << ", horizontal "
              << fused_error_.horizontal.mean () << ", vertical "
              << fused_error_.vertical.mean () << " | detections "
              << detections_ << " (rejected " << detections_rejected_
              << ", late " << detections_late_ << ")" << std::endl;
  }

  void
  publish_overlay ()
  {
    std::ostringstream o;
    o << std::fixed << std::setprecision (2)
      << "<b>Pole tracking: UWB + camera" << (opt_.use_imu ? " + IMU" : "")
      << (opt_.use_gps ? " + GPS" : "") << "</b><br>"
      << "error now: <b>" << fused_error_.total.last () << " m</b><br>"
      << "mean " << fused_error_.total.mean () << " m | horizontal "
      << fused_error_.horizontal.mean () << " m | vertical "
      << fused_error_.vertical.mean () << " m<br>"
      << "<font color=\"#aaaaaa\">camera ray + UWB alone: "
      << raw_fix_error_.total.mean () << " m<br>"
      << "camera " << (opt_.track_camera ? "tracking" : "fixed")
      << ", drone in view " << std::setprecision (0) << in_view_percent ()
      << " %<br>" << std::setprecision (2)
      << "detections " << detections_ << " | sim time "
      << std::setprecision (1) << sim_t_ << " s</font>";
    gz::msgs::StringMsg msg;
    msg.set_data (o.str ());
    overlay_pub_.Publish (msg);
  }

  [[nodiscard]] double
  in_view_percent () const
  {
    return frames_scored_ == 0 ? 0.0
                               : 100.0 * static_cast<double> (frames_in_view_)
                                     / static_cast<double> (frames_scored_);
  }

  void
  print_summary () const
  {
    const auto row = [] (const char *name, const error_stats &s)
      {
        std::cout << "  " << std::left << std::setw (34) << name << std::right
                  << std::fixed << std::setprecision (3) << std::setw (8)
                  << s.mean () << std::setw (8) << s.rms () << std::setw (8)
                  << s.percentile (0.95) << std::setw (8) << s.max ()
                  << std::setw (9) << s.count () << "\n";
      };
    std::cout << "\n=== error against ground truth over " << std::fixed
              << std::setprecision (1) << sim_t_ << " s (metres), sensors: uwb"
              << ", camera" << (opt_.use_imu ? ", imu" : "")
              << (opt_.use_gps ? ", gps" : "") << " ===\n"
              << "  " << std::left << std::setw (34) << "" << std::right
              << std::setw (8) << "mean" << std::setw (8) << "rms"
              << std::setw (8) << "p95" << std::setw (8) << "max"
              << std::setw (9) << "samples" << "\n";
    row ("fused estimate, 3d", fused_error_.total);
    row ("fused estimate, horizontal", fused_error_.horizontal);
    row ("fused estimate, vertical", fused_error_.vertical);
    row ("camera ray + uwb alone, 3d", raw_fix_error_.total);
    if (opt_.use_gps)
      {
        row ("raw gps fix, 3d", gps_error_.total);
      }
    std::cout << "  camera " << (opt_.track_camera ? "tracking" : "fixed")
              << ", drone in view " << std::setprecision (1)
              << in_view_percent () << " % of the time\n";
    std::cout << "  detections: " << detections_ << " (rejected "
              << detections_rejected_ << ", too late " << detections_late_
              << ", resets " << resets_ << ", diverged " << diverged_
              << "), uwb ranges: " << ranges_
              << " (rejected " << ranges_rejected_ << ")";
    if (opt_.use_gps)
      {
        std::cout << ", gps fixes: " << gps_fixes_ << " (rejected "
                  << gps_rejected_ << ")";
      }
    std::cout << std::endl;
  }
};
} // namespace

int
main (int argc, char *argv[])
{
  std::optional<options> opt;
  try
    {
      opt = parse_options (argc, argv);
    }
  catch (const std::exception &e)
    {
      std::cerr << e.what () << "\n";
    }
  if (!opt)
    {
      print_usage (argv[0]);
      return 2;
    }

  std::signal (SIGINT, handle_signal);
  std::signal (SIGTERM, handle_signal);

  pole_tracking_sim sim (*opt);
  if (!sim.start ())
    {
      return 1;
    }
  sim.run ();
  return 0;
}
