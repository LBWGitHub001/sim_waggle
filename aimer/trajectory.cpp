#include "trajectory.hpp"

#include <cmath>

namespace common
{
constexpr double kG = 9.7833;

BulletTrajectory::BulletTrajectory(double v0, double d, double h)
{
  if (v0 < 1e-6) {
    unsolvable = true;
    return;
  }
  const double a = kG * d * d / (2.0 * v0 * v0);
  const double b = -d;
  const double c = a + h;
  const double delta = b * b - 4.0 * a * c;

  if (delta < 0.0 || std::abs(a) < 1e-12) {
    unsolvable = true;
    return;
  }

  unsolvable = false;
  const double tan1 = (-b + std::sqrt(delta)) / (2.0 * a);
  const double tan2 = (-b - std::sqrt(delta)) / (2.0 * a);
  const double pitch1 = std::atan(tan1);
  const double pitch2 = std::atan(tan2);
  const double t1 = d / (v0 * std::cos(pitch1));
  const double t2 = d / (v0 * std::cos(pitch2));

  if (t1 < t2) {
    pitch = pitch1;
    fly_time = t1;
  } else {
    pitch = pitch2;
    fly_time = t2;
  }
}

}  // namespace common
