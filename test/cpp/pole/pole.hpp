#ifndef POLE_TEST_HPP
#define POLE_TEST_HPP

#include "check.hpp"
#include "pole_estimator.hpp"

void
pole_estimator_test ()
{
  const pole_camera camera{};

  // The image center looks along the camera axis: back along -x, 10 deg up.
  const auto center = camera.ray (960.0, 540.0);
  check_near (center[0], -std::cos (0.1745), 1e-6, "center ray x");
  check_near (center[1], 0.0, 1e-6, "center ray y");
  check_near (center[2], std::sin (0.1745), 1e-6, "center ray up");
  // Facing -x, the right of the image is +y; the top of the image is up.
  check (camera.ray (1500.0, 540.0)[1] > 0.1, "image right is +y");
  check (camera.ray (960.0, 100.0)[2] > center[2], "image top is up");

  // pixel () undoes ray (), also with the pan-tilt mount turned.
  pole_camera turned = camera;
  turned.yaw_rad = 2.2;
  turned.pitch_rad = -0.6;
  const vec3 far = camera.position;
  const vec3 r = turned.ray (300.0, 800.0);
  const auto px = turned.pixel (
      { far[0] + 20 * r[0], far[1] + 20 * r[1], far[2] + 20 * r[2] });
  check (px.has_value (), "pixel in front");
  check_near ((*px)[0], 300.0, 1e-6, "pixel u round trip");
  check_near ((*px)[1], 800.0, 1e-6, "pixel v round trip");
  check (!turned.pixel ({ far[0] - 20 * r[0], far[1] - 20 * r[1],
                          far[2] - 20 * r[2] }),
         "behind the camera");
  check (!camera.sees ({ 40.0, 0.0, 3.0 }), "drone behind the pole");

  const vec3 truth{ 0.0, 8.0, 8.0 };
  const vec3 offset = minus (truth, camera.position);
  const double range = norm (offset);
  const vec3 ray = normalized (offset);

  // Start 3 m off: range and bearing together must pull the state to the
  // true point, each one only along its own direction.
  pole_estimator estimator{ {}, camera.position };
  estimator.initialize (0.0, { 2.0, 6.0, 9.0 }, 5.0);
  for (int i = 1; i <= 20; ++i)
    {
      estimator.advance_to (i * 0.01);
      check (estimator.add_range (range), "range accepted");
      check (estimator.add_bearing (ray), "bearing accepted");
    }
  const auto p = estimator.position ();
  check_near (p[0], truth[0], 0.1, "pole fix x");
  check_near (p[1], truth[1], 0.1, "pole fix y");
  check_near (p[2], truth[2], 0.1, "pole fix z");

  // Range alone only pins the distance: the state ends up on the sphere.
  pole_estimator range_only{ {}, camera.position };
  range_only.initialize (0.0, { 2.0, 6.0, 9.0 }, 5.0);
  for (int i = 1; i <= 20; ++i)
    {
      range_only.advance_to (i * 0.01);
      (void)range_only.add_range (range);
    }
  check_near (norm (minus (range_only.position (), camera.position)), range,
              0.1, "range alone reaches the sphere");

  // A wrong detection far from the converged state is gated out.
  check (!estimator.add_bearing (camera.ray (200.0, 900.0)),
         "false detection rejected");
  check (!estimator.add_range (range + 5.0), "range outlier rejected");
}

#endif // !POLE_TEST_HPP
