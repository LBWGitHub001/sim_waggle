#include "kinematics.hpp"

#include <cmath>

namespace sim
{
namespace
{
double limit_rad(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// 绕 Z 轴旋转 yaw 弧度的旋转矩阵
Eigen::Matrix3d rot_z(double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  Eigen::Matrix3d r;
  r << c, -s, 0,
       s,  c, 0,
       0,  0, 1;
  return r;
}
}  // namespace

KinematicsEngine::KinematicsEngine(const common::SimulationConfig & cfg)
: cfg_(cfg)
{
  reset();
}

void KinematicsEngine::reset()
{
  gt_ = common::GroundTruth{};
  gt_.t = 0.0;

  // 积分量初值
  gt_.chassis.vel = Eigen::Vector3d(cfg_.chassis_vx, cfg_.chassis_vy, 0.0);

  gt_.target.center = cfg_.target_init_pos;
  gt_.target.vel = cfg_.target_vel;
  gt_.target.radius = cfg_.target_radius;
  gt_.target.armor_num = cfg_.armor_num;
  gt_.target.armors_pos.resize(cfg_.armor_num);
  gt_.target.armors_yaw.resize(cfg_.armor_num);

  // t = 0 时刻的派生量：直接计算，不做任何隐藏步进
  update_derived();
}

void KinematicsEngine::step(double dt)
{
  gt_.t += dt;
  integrate_chassis(dt);
  integrate_target(dt);
  update_derived();
}

void KinematicsEngine::integrate_chassis(double dt)
{
  gt_.chassis.w = cfg_.chassis_w;
  gt_.chassis.pos += gt_.chassis.vel * dt;
  gt_.chassis.yaw = limit_rad(gt_.chassis.yaw + gt_.chassis.w * dt);
}

void KinematicsEngine::integrate_target(double dt)
{
  gt_.target.w = cfg_.target_w;
  gt_.target.vel = cfg_.target_vel;
  gt_.target.center += gt_.target.vel * dt;
  gt_.target.yaw = limit_rad(gt_.target.yaw + gt_.target.w * dt);
}

void KinematicsEngine::update_derived()
{
  update_gimbal();
  update_armors();
  select_visible_armor();
}

void KinematicsEngine::update_gimbal()
{
  // 偏置杆臂的世界系矢量；云台线速度 = 底盘平移 + 杆臂公转 ω × r
  const Eigen::Vector3d r_world = rot_z(gt_.chassis.yaw) * cfg_.gimbal_offset;
  gt_.gimbal.pos = gt_.chassis.pos + r_world;
  gt_.gimbal.vel = gt_.chassis.vel + Eigen::Vector3d(0.0, 0.0, gt_.chassis.w).cross(r_world);

  // 云台始终平滑对准目标中心
  const Eigen::Vector3d to_target = gt_.target.center - gt_.gimbal.pos;
  gt_.gimbal.yaw = std::atan2(to_target.y(), to_target.x());
  gt_.gimbal.pitch = 0.0;
  gt_.gimbal.R_gimbal2world = rot_z(gt_.gimbal.yaw);
}

void KinematicsEngine::update_armors()
{
  for (int i = 0; i < cfg_.armor_num; i++) {
    const double yaw_i = limit_rad(gt_.target.yaw + i * 2.0 * M_PI / cfg_.armor_num);
    gt_.target.armors_yaw[i] = yaw_i;
    gt_.target.armors_pos[i] = gt_.target.center + Eigen::Vector3d(
      -cfg_.target_radius * std::cos(yaw_i),
      -cfg_.target_radius * std::sin(yaw_i),
      0.0);
  }
}

void KinematicsEngine::select_visible_armor()
{
  // 装甲板法向与"装甲板指向云台"视线方向点积最大者，即正面最朝向云台的装甲板
  int best_id = 0;
  double max_dot = -1e10;
  for (int i = 0; i < cfg_.armor_num; i++) {
    const Eigen::Vector3d los = (gt_.gimbal.pos - gt_.target.armors_pos[i]).normalized();
    const Eigen::Vector3d normal(
      -std::cos(gt_.target.armors_yaw[i]), -std::sin(gt_.target.armors_yaw[i]), 0.0);
    const double dot = normal.dot(los);
    if (dot > max_dot) {
      max_dot = dot;
      best_id = i;
    }
  }
  gt_.visible_armor_id = best_id;
}

}  // namespace sim
