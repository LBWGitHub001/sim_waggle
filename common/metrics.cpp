#include "metrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace common
{
namespace
{

/// 角度差归一化到 (-pi, pi]
double normalize_angle(double angle)
{
  constexpr double k2Pi = 2.0 * M_PI;
  double a = std::fmod(angle + M_PI, k2Pi);
  if (a < 0.0) a += k2Pi;
  return a - M_PI;
}

/// 估计装甲板与真值装甲板的平均最近邻距离（4 块板的空间拟合误差）
double armor_fit_error(
  const TargetEstimate & est,
  const TargetState & target)
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

}  // namespace

void SimulationMetrics::reset()
{
  latest_ = MetricsResult{};
  samples_ = 0;
  sum_sq_pos_ = sum_sq_vel_ = 0.0;
  sum_sq_pos_rel_ = sum_sq_vel_rel_ = 0.0;
  sum_pos_ = sum_vel_ = 0.0;
  max_pos_ = max_vel_ = 0.0;
}

MetricsResult SimulationMetrics::update(
  const GroundTruth & gt,
  const TargetEstimate & est,
  double bullet_speed)
{
  MetricsResult r;

  // 估计未成形（x 维数不足）时不做误差评估，避免 0 值污染曲线
  if (est.x.size() < 11) {
    latest_ = r;
    return r;
  }

  // ---------------- 1. 绝对误差：估计 vs 世界系真值 ----------------
  const Eigen::Vector3d & true_p = gt.target.center;
  const Eigen::Vector3d & true_v = gt.target.vel;

  r.err_x = est.center.x() - true_p.x();
  r.err_y = est.center.y() - true_p.y();
  r.err_vx = est.velocity.x() - true_v.x();
  r.err_vy = est.velocity.y() - true_v.y();

  r.pos_err_norm = std::hypot(r.err_x, r.err_y);
  r.vel_err_norm = std::hypot(r.err_vx, r.err_vy);

  // ---------------- 2. 相对误差：估计 vs 云台系相对真值 ----------------
  // 观测 xyz_in_world = armor_pos - gimbal.pos，EKF 状态活在这一坐标系下
  const Eigen::Vector3d rel_p = true_p - gt.gimbal.pos;
  const Eigen::Vector3d rel_v = true_v - gt.gimbal.vel;

  r.rel_err_x = est.center.x() - rel_p.x();
  r.rel_err_y = est.center.y() - rel_p.y();
  r.rel_err_vx = est.velocity.x() - rel_v.x();
  r.rel_err_vy = est.velocity.y() - rel_v.y();

  r.rel_pos_err_norm = std::hypot(r.rel_err_x, r.rel_err_y);
  r.rel_vel_err_norm = std::hypot(r.rel_err_vx, r.rel_err_vy);

  // ---------------- 3. 姿态与角速度 ----------------
  r.err_yaw = normalize_angle(est.yaw - gt.target.yaw);
  r.err_w = est.w - gt.target.w;

  // ---------------- 4. 装甲板几何一致性 ----------------
  r.armor_err_norm = armor_fit_error(est, gt.target);

  // ---------------- 5. 弹道超前与一致性 ----------------
  const double dist_to_gimbal = (est.center - gt.gimbal.pos).norm();
  const double v_b = (bullet_speed > 1.0) ? bullet_speed : 25.0;
  r.flight_time = dist_to_gimbal / v_b;
  r.aim_lead_bias = r.vel_err_norm * r.flight_time;
  r.rel_aim_lead_bias = r.rel_vel_err_norm * r.flight_time;
  r.nis = est.last_nis;

  // ---------------- 6. 累计统计 ----------------
  samples_++;
  sum_sq_pos_ += r.pos_err_norm * r.pos_err_norm;
  sum_sq_vel_ += r.vel_err_norm * r.vel_err_norm;
  sum_sq_pos_rel_ += r.rel_pos_err_norm * r.rel_pos_err_norm;
  sum_sq_vel_rel_ += r.rel_vel_err_norm * r.rel_vel_err_norm;
  sum_pos_ += r.pos_err_norm;
  sum_vel_ += r.vel_err_norm;
  max_pos_ = std::max(max_pos_, r.pos_err_norm);
  max_vel_ = std::max(max_vel_, r.vel_err_norm);

  const double inv_n = 1.0 / static_cast<double>(samples_);
  r.samples = samples_;
  r.rmse_pos = std::sqrt(sum_sq_pos_ * inv_n);
  r.rmse_vel = std::sqrt(sum_sq_vel_ * inv_n);
  r.rmse_pos_rel = std::sqrt(sum_sq_pos_rel_ * inv_n);
  r.rmse_vel_rel = std::sqrt(sum_sq_vel_rel_ * inv_n);
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
