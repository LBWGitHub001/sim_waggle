#ifndef SIM__KINEMATICS_HPP
#define SIM__KINEMATICS_HPP

#include "types.hpp"

namespace sim
{

/// 物理与运动学仿真引擎
/// 模拟自车底盘平移、小陀螺自转与偏置公转云台；
/// 模拟敌车目标平移与多装甲板旋转。
///
/// 内部状态分两类，方便调试时区分：
/// - 积分量：底盘 pos/yaw、目标 center/yaw（只在 integrate_* 中随时间累积）
/// - 派生量：云台位姿/速度、装甲板位姿、可见装甲板（update_derived() 纯推导，
///   任意时刻调用都得到一致结果，无隐藏时序依赖）
class KinematicsEngine
{
public:
  explicit KinematicsEngine(const common::SimulationConfig & cfg = common::SimulationConfig{});

  // 重置仿真状态：t 严格归零，派生量直接计算（无任何隐藏步进）
  void reset();

  // 动力学推演步进 dt 秒
  void step(double dt);

  // 获取当前物理真值
  const common::GroundTruth & ground_truth() const { return gt_; }

  // 动态修改参数（调试时用）
  void set_chassis_w(double w) { cfg_.chassis_w = w; }
  void set_gimbal_offset(const Eigen::Vector3d & offset) { cfg_.gimbal_offset = offset; }
  void set_target_vel(const Eigen::Vector3d & vel) { cfg_.target_vel = vel; }

private:
  void integrate_chassis(double dt);  // 底盘平移 + 自转积分
  void integrate_target(double dt);   // 目标平移 + 自转积分
  void update_derived();              // 由积分量推导全部派生量
  void update_gimbal();               // 云台位置 / 公转线速度 / 瞄准朝向
  void update_armors();               // 目标各装甲板世界位姿
  void select_visible_armor();        // 法向最朝向云台的装甲板

  common::SimulationConfig cfg_;
  common::GroundTruth gt_;
};

}  // namespace sim

#endif  // SIM__KINEMATICS_HPP
