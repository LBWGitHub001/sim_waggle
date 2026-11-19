#ifndef COMMON__METRICS_HPP
#define COMMON__METRICS_HPP

#include "types.hpp"

namespace common
{

struct MetricsResult
{
  double err_x = 0.0;
  double err_y = 0.0;
  double err_vx = 0.0;
  double err_vy = 0.0;
  double pos_err_norm = 0.0;
  double vel_err_norm = 0.0;   // 伪速度幅值 (m/s)
  double flight_time = 0.0;    // 弹丸飞行时间 (s)
  double aim_lead_bias = 0.0;  // 弹道超前瞄准偏差 (m)
  double nis = 0.0;            // 归一化新息平方
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

private:
  MetricsResult latest_;
};

// 由估计快照计算弹道超前瞄准落点
// （公式与 BaselineTracker::predict_aim_point 一致，供可视化线程使用，
//   避免跨线程访问 tracker 内部状态）
Eigen::Vector3d compute_aim_lead_point(const TargetEstimate & est, double flight_time);

}  // namespace common

#endif  // COMMON__METRICS_HPP
