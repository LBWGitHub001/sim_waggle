#include "metrics.hpp"

#include <cmath>
#include <algorithm>

namespace common
{

MetricsResult SimulationMetrics::update(
  const GroundTruth & gt,
  const TargetEstimate & est,
  double bullet_speed)
{
  MetricsResult r;

  if (est.x.size() >= 4) {
    // 世界系原点在云台：观测 xyz_in_world = armor_pos - gimbal.pos 就是该坐标系下的量，
    // 估计状态也活在这个系里，所以真值必须同样取"目标相对云台"的位姿
    // （其运动 = 目标自身运动 + 云台偏置公转，两者混在一起）。
    const Eigen::Vector3d true_p = gt.target.center - gt.gimbal.pos;
    const Eigen::Vector3d true_v = gt.target.vel - gt.gimbal.vel;

    r.err_x = est.center.x() - true_p.x();
    r.err_y = est.center.y() - true_p.y();
    r.err_vx = est.velocity.x() - true_v.x();
    r.err_vy = est.velocity.y() - true_v.y();

    r.pos_err_norm = std::hypot(r.err_x, r.err_y);
    r.vel_err_norm = std::hypot(r.err_vx, r.err_vy);

    // est.center 本身已是云台系坐标，弹道距离直接取模长
    double dist_to_gimbal = est.center.norm();
    double v_b = (bullet_speed > 1.0) ? bullet_speed : 25.0;
    r.flight_time = dist_to_gimbal / v_b;

    // 偏置伪速度对超前预测落点的脱靶量
    r.aim_lead_bias = r.vel_err_norm * r.flight_time;
  }

  r.nis = est.last_nis;
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
