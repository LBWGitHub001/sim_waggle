#include "kinematics.hpp"

#include <cmath>

#include "frames.hpp"

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

  gt_.chassis.vel = Eigen::Vector3d(cfg_.chassis_vx, cfg_.chassis_vy, 0.0);

  gt_.target.center = cfg_.target_init_pos;
  gt_.target.vel = cfg_.target_vel;
  gt_.target.radius = cfg_.target_radius;
  gt_.target.l = cfg_.target_l;
  gt_.target.h = cfg_.target_h;
  gt_.target.armor_num = cfg_.armor_num;
  gt_.target.armors_pos.resize(cfg_.armor_num);
  gt_.target.armors_yaw.resize(cfg_.armor_num);

  last_gimbal_vel_.setZero();
  origin_acc_.setZero();
  last_dt_ = 0.0;

  update_derived();
}

void KinematicsEngine::step(double dt)
{
  last_dt_ = dt;
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
  update_origin_acc();
}

void KinematicsEngine::update_gimbal()
{
  const Eigen::Vector3d r_odom = rot_z(gt_.chassis.yaw) * cfg_.gimbal_offset;
  gt_.gimbal.pos = gt_.chassis.pos + r_odom;
  gt_.gimbal.vel = gt_.chassis.vel + Eigen::Vector3d(0.0, 0.0, gt_.chassis.w).cross(r_odom);
  gt_.gimbal.yaw = 0.0;
  gt_.gimbal.pitch = 0.0;
  gt_.gimbal.R_gimbal2world = Eigen::Matrix3d::Identity();

  const Eigen::Vector3d to_target = gt_.target.center - gt_.gimbal.pos;
  gt_.camera.pos = gt_.gimbal.pos;
  gt_.camera.yaw = std::atan2(to_target.y(), to_target.x());
  gt_.camera.pitch = 0.0;
  gt_.camera.R_camera2odom = rot_z(gt_.camera.yaw);
}

void KinematicsEngine::update_armors()
{
  const Eigen::Matrix3d R_robot2odom = rot_z(gt_.target.yaw);
  const int n = cfg_.armor_num;
  const double r = cfg_.target_radius;
  const double l = cfg_.target_l;
  const double h = cfg_.target_h;
  for (int i = 0; i < n; i++) {
    const Eigen::Vector3d p_robot = common::armor_in_robot(i, n, r, l, h);
    gt_.target.armors_pos[i] = gt_.target.center + R_robot2odom * p_robot;
    gt_.target.armors_yaw[i] = limit_rad(gt_.target.yaw + common::armor_yaw_in_robot(i, n));
  }
}

void KinematicsEngine::select_visible_armor()
{
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

void KinematicsEngine::update_origin_acc()
{
  if (last_dt_ > 1e-12) {
    origin_acc_ = (gt_.gimbal.vel - last_gimbal_vel_) / last_dt_;
  } else {
    origin_acc_.setZero();
  }
  last_gimbal_vel_ = gt_.gimbal.vel;
}

}  // namespace sim
