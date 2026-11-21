#ifndef SIM__KINEMATICS_HPP
#define SIM__KINEMATICS_HPP

#include "types.hpp"

namespace sim
{

/// 物理与运动学仿真引擎
/// 模拟自车底盘平移、小陀螺自转与偏置公转云台；
/// 模拟敌车目标平移与多装甲板旋转。
///
/// 积分在辅助惯性系中进行（初值：底盘中心在原点，目标在 target_init_pos）。
/// 对外 ground_truth() 已变换到世界系：原点 = 云台中心，轴向固定不转。
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

  // 当前物理真值：世界系几何 + target_inertial（场地敌车）
  const common::GroundTruth & ground_truth() const { return world_; }

  // 动态修改参数（调试时用）
  void set_chassis_w(double w) { cfg_.chassis_w = w; }
  void set_gimbal_offset(const Eigen::Vector3d & offset) { cfg_.gimbal_offset = offset; }
  void set_target_vel(const Eigen::Vector3d & vel) { cfg_.target_vel = vel; }

private:
  void integrate_chassis(double dt);  // 底盘平移 + 自转积分
  void integrate_target(double dt);   // 目标平移 + 自转积分
  void update_derived();              // 由积分量推导全部派生量
  void update_gimbal();               // 云台位置 / 公转线速度 / 瞄准朝向
  void update_armors();               // 目标各装甲板位姿
  void select_visible_armor();        // 法向最朝向云台的装甲板
  void express_in_world_frame();      // 辅助惯性系 → 世界系（原点=云台）

  common::SimulationConfig cfg_;
  common::GroundTruth gt_;     // 辅助惯性系，仅用于积分
  common::GroundTruth world_;  // 对外世界系
};

}  // namespace sim

#endif  // SIM__KINEMATICS_HPP
