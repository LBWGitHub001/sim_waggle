#ifndef COMMON__METRICS_HPP
#define COMMON__METRICS_HPP

#include <cstdint>

#include "types.hpp"

namespace common
{

/// 估计误差评估结果（每帧刷新；累计统计自启动以来）
///
/// 两套误差口径，切勿混用：
///   - 绝对 (abs)：估计 vs 世界系真值。云台绕底盘偏置公转带来的平移会被
///     计入误差，因此这一项包含"伪速度/自车运动"的贡献 —— 正是本项目要
///     暴露的偏置影响。
///   - 相对 (rel)：估计 vs 云台系相对真值 (target - gimbal)。观测
///     xyz_in_world 恰为该坐标系下的量，故此项反映 EKF 自身的跟踪精度。
struct MetricsResult
{
  // --- 绝对误差：估计 vs 世界系真值 ---
  double err_x = 0.0;
  double err_y = 0.0;
  double err_vx = 0.0;
  double err_vy = 0.0;
  double pos_err_norm = 0.0;   // |est.center - gt.target.center|  (m)
  double vel_err_norm = 0.0;   // |est.vel - gt.target.vel|        (m/s, 伪速度幅值)

  // --- 相对误差：估计 vs 云台系相对真值 ---
  double rel_err_x = 0.0;
  double rel_err_y = 0.0;
  double rel_err_vx = 0.0;
  double rel_err_vy = 0.0;
  double rel_pos_err_norm = 0.0;
  double rel_vel_err_norm = 0.0;

  // --- 目标姿态 ---
  double err_yaw = 0.0;        // 归一化到 (-pi, pi] (rad)
  double err_w = 0.0;          // (rad/s)

  // --- 装甲板几何一致性 ---
  double armor_err_norm = 0.0; // 拟合装甲板到最近真值装甲板的平均距离 (m)

  // --- 弹道与滤波器一致性 ---
  double flight_time = 0.0;      // 弹丸飞行时间 (s)
  double aim_lead_bias = 0.0;    // 绝对速度误差折算的超前瞄准脱靶量 (m) = vel_err * T
  double rel_aim_lead_bias = 0.0;  // 相对速度误差折算的脱靶量 (m) = rel_vel_err * T
  double nis = 0.0;              // 归一化新息平方

  // --- 累计统计（自启动以来） ---
  uint64_t samples = 0;
  double rmse_pos = 0.0;
  double rmse_vel = 0.0;
  double rmse_pos_rel = 0.0;
  double rmse_vel_rel = 0.0;
  double mean_pos = 0.0;
  double mean_vel = 0.0;
  double max_pos = 0.0;
  double max_vel = 0.0;
};

class SimulationMetrics
{
public:
  SimulationMetrics() = default;

  MetricsResult update(
    const GroundTruth & gt,
    const TargetEstimate & est,
    double bullet_speed = 25.0);

  const MetricsResult & latest() const { return latest_; }

  void reset();

private:
  MetricsResult latest_;

  // 累计量（避免浮点累计误差，均以和/平方和形式保存）
  uint64_t samples_ = 0;
  double sum_sq_pos_ = 0.0;
  double sum_sq_vel_ = 0.0;
  double sum_sq_pos_rel_ = 0.0;
  double sum_sq_vel_rel_ = 0.0;
  double sum_pos_ = 0.0;
  double sum_vel_ = 0.0;
  double max_pos_ = 0.0;
  double max_vel_ = 0.0;
};

// 由估计快照计算弹道超前瞄准落点
// （公式与 BaselineTracker::predict_aim_point 一致，供可视化线程使用，
//   避免跨线程访问 tracker 内部状态）
Eigen::Vector3d compute_aim_lead_point(const TargetEstimate & est, double flight_time);

}  // namespace common

#endif  // COMMON__METRICS_HPP
