#include "metrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace common
{
namespace
{

double normalize_angle(double angle)
{
  constexpr double k2Pi = 2.0 * M_PI;
  double a = std::fmod(angle + M_PI, k2Pi);
  if (a < 0.0) a += k2Pi;
  return a - M_PI;
}

double armor_fit_error(const TargetEstimate & est, const TargetState & target)
{
  const auto & plates = est.armor_xyza_list;
  if (plates.empty() || target.armors_pos.empty()) return 0.0;

  double sum = 0.0;
  for (const auto & p : plates) {
    const Eigen::Vector3d pe(p[0], p[1], p[2]);
    double best = std::numeric_limits<double>::max();
    for (const auto & gt_plate : target.armors_pos) {
      best = std::min(best, (pe - gt_plate).norm());
    }
    sum += best;
  }
  return sum / static_cast<double>(plates.size());
}

TargetEstimate truth_as_estimate(const TargetState & t)
{
  TargetEstimate e;
  e.center = t.center;
  e.velocity = t.vel;
  e.yaw = t.yaw;
  e.radius = t.radius;
  e.l = t.l;
  e.h = t.h;
  return e;
}

}  // namespace

void SimulationMetrics::reset()
{
  latest_ = MetricsResult{};
  samples_ = 0;
  sum_sq_pos_ = sum_sq_vel_ = 0.0;
  sum_pos_ = sum_vel_ = 0.0;
  max_pos_ = max_vel_ = 0.0;
}

MetricsResult SimulationMetrics::update(
  const GroundTruth & gt,
  const TargetEstimate & est,
  double bullet_speed)
{
  MetricsResult r;

  if (est.x.size() < 11) {
    latest_ = r;
    return r;
  }

  const TargetState & truth = gt.target;

  r.err_x = est.center.x() - truth.center.x();
  r.err_y = est.center.y() - truth.center.y();
  r.err_z = est.center.z() - truth.center.z();
  r.err_vx = est.velocity.x() - truth.vel.x();
  r.err_vy = est.velocity.y() - truth.vel.y();
  r.err_vz = est.velocity.z() - truth.vel.z();
  r.err_yaw = normalize_angle(est.yaw - truth.yaw);
  r.err_w = est.w - truth.w;
  r.err_r = est.radius - truth.radius;
  r.err_l = est.l - truth.l;
  r.err_h = est.h - truth.h;

  r.pos_err_norm = (est.center - truth.center).norm();
  r.vel_err_norm = (est.velocity - truth.vel).norm();
  r.armor_err_norm = armor_fit_error(est, truth);

  const double v_b = (bullet_speed > 1.0) ? bullet_speed : 25.0;
  r.flight_time = est.center.norm() / v_b;
  const Eigen::Vector3d aim_kal = compute_aim_lead_point(est, r.flight_time);
  const Eigen::Vector3d aim_true = compute_aim_lead_point(truth_as_estimate(truth), r.flight_time);
  r.aim_lead_bias = (aim_kal - aim_true).norm();
  r.nis = est.last_nis;

  samples_++;
  sum_sq_pos_ += r.pos_err_norm * r.pos_err_norm;
  sum_sq_vel_ += r.vel_err_norm * r.vel_err_norm;
  sum_pos_ += r.pos_err_norm;
  sum_vel_ += r.vel_err_norm;
  max_pos_ = std::max(max_pos_, r.pos_err_norm);
  max_vel_ = std::max(max_vel_, r.vel_err_norm);

  const double inv_n = 1.0 / static_cast<double>(samples_);
  r.samples = samples_;
  r.rmse_pos = std::sqrt(sum_sq_pos_ * inv_n);
  r.rmse_vel = std::sqrt(sum_sq_vel_ * inv_n);
  r.mean_pos = sum_pos_ * inv_n;
  r.mean_vel = sum_vel_ * inv_n;
  r.max_pos = max_pos_;
  r.max_vel = max_vel_;

  latest_ = r;
  return r;
}

Eigen::Vector3d compute_aim_lead_point(const TargetEstimate & est, double flight_time)
{
  const Eigen::Vector3d armor_pos = est.center + Eigen::Vector3d(
    -est.radius * std::cos(est.yaw), -est.radius * std::sin(est.yaw), 0.0);
  return armor_pos + est.velocity * flight_time;
}

}  // namespace common
