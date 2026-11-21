#ifndef SIM__KINEMATICS_HPP
#define SIM__KINEMATICS_HPP

#include "types.hpp"

namespace sim
{

/// 物理与运动学仿真引擎
/// 积分在 odom（场地惯性系，中间件）：底盘小陀螺、偏置云台、敌车 robot 自转。
/// 装甲板绑在 robot 系，经 Rz(robot.yaw) 变到 odom。
/// gimbal 原点跟云台走、轴向与 odom 平行；camera 同原点、轴向 look-at。
/// ground_truth() 返回 odom 位姿；日常滤波请用 common::express_target_in_gimbal。
///
/// 内部状态分两类：
/// - 积分量：底盘 pos/yaw、目标 center/yaw（只在 integrate_* 中随时间累积）
/// - 派生量：云台位姿/速度、装甲板位姿、可见装甲板（update_derived() 纯推导）
class KinematicsEngine
{
public:
  explicit KinematicsEngine(const common::SimulationConfig & cfg = common::SimulationConfig{});

  void reset();
  void step(double dt);

  // odom 真值（给 TF / 中间件）
  const common::GroundTruth & ground_truth() const { return gt_; }

  // 云台原点在 odom 的惯性加速度（速度差分）；Control_inje 用 -R^T * acc 作控制量
  const Eigen::Vector3d & origin_acc() const { return origin_acc_; }

  void set_chassis_w(double w) { cfg_.chassis_w = w; }
  void set_gimbal_offset(const Eigen::Vector3d & offset) { cfg_.gimbal_offset = offset; }
  void set_target_vel(const Eigen::Vector3d & vel) { cfg_.target_vel = vel; }

private:
  void integrate_chassis(double dt);
  void integrate_target(double dt);
  void update_derived();
  void update_gimbal();
  void update_armors();
  void select_visible_armor();
  void update_origin_acc();

  common::SimulationConfig cfg_;
  common::GroundTruth gt_;  // odom
  Eigen::Vector3d last_gimbal_vel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d origin_acc_ = Eigen::Vector3d::Zero();
  double last_dt_ = 0.0;
};

}  // namespace sim

#endif  // SIM__KINEMATICS_HPP
