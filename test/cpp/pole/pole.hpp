#ifndef POLE_TEST_HPP
#define POLE_TEST_HPP

#include "check.hpp"
#include "pole_estimator.hpp"

#include <limits>

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

  // Measurements that are not numbers are rejected and leave it healthy.
  const double nan = std::numeric_limits<double>::quiet_NaN ();
  check (!estimator.add_range (nan), "nan range rejected");
  check (!estimator.add_bearing ({ nan, 0.0, 1.0 }), "nan ray rejected");
  check (!estimator.add_gps ({ nan, 0.0, 0.0 }, 1.0, 1.0),
         "nan gps rejected");
  check (estimator.healthy (), "healthy after nan input");

  // A drone straight above the pole: the ray is vertical.
  const vec3 above{ camera.position[0], camera.position[1],
                    camera.position[2] + 10.0 };
  pole_estimator overhead{ {}, camera.position };
  overhead.initialize (0.0, { above[0] + 0.5, above[1], above[2] }, 2.0);
  for (int i = 1; i <= 20; ++i)
    {
      overhead.advance_to (i * 0.01);
      (void)overhead.add_bearing ({ 0.0, 0.0, 1.0 });
      (void)overhead.add_range (10.0);
    }
  check (overhead.healthy (), "vertical ray keeps it finite");
  check_near (norm (minus (overhead.position (), above)), 0.0, 0.1,
              "vertical ray fix");

  // An estimate sitting on the anchor has no bearing scale; still finite.
  pole_estimator on_anchor{ {}, camera.position };
  on_anchor.initialize (0.0, camera.position, 1.0);
  (void)on_anchor.add_bearing (center);
  check (on_anchor.healthy (), "estimate on the anchor stays finite");

  on_anchor.reset ();
  check (!on_anchor.initialized (), "reset forgets the state");
}

#endif // !POLE_TEST_HPP
