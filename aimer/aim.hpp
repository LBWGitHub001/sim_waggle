#ifndef COMMON__AIM_HPP
#define COMMON__AIM_HPP

#include "types.hpp"

namespace common
{

struct AimPoint
{
  bool valid = false;
  Eigen::Vector4d xyza = Eigen::Vector4d::Zero();
  int id = -1;
};

/// 11 维 CV 外推，并按 r/l/h 重建装甲板。不改协方差。
TargetEstimate propagate_estimate(const TargetEstimate & est, double dt);

Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id, int armor_num);

/// gimbal 系 GT → 11 维状态（jumped=true，装甲由 r/l/h 重建）。给 Aimer::aim 用。
TargetEstimate estimate_from_ground_truth(const GroundTruth & gt);

/// 弹道 Aimer：coming/leaving 选板 + 飞行时间迭代。输出无约束射击角。
class Aimer
{
public:
  explicit Aimer(AimerParams params = {});

  AimPoint choose_aim_point(const TargetEstimate & est);
  /// 选板 + 一次弹道，无飞行时间迭代。不可解则 valid=false。
  AimPoint yaw_pitch(const TargetEstimate & est, double bullet_speed, Eigen::Vector2d & yp);
  GimbalCommand aim(TargetEstimate est, double bullet_speed);
  const AimerParams & params() const { return params_; }

private:
  AimerParams params_;
  int lock_id_ = -1;
};

}  // namespace common

#endif
