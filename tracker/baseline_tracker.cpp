#include "baseline_tracker.hpp"

#include <cmath>

namespace tracker
{

BaselineTracker::BaselineTracker(
  double target_radius,
  int armor_num,
  const auto_aim::EkfParams & ekf_params,
  const Eigen::VectorXd & p0)
: target_radius_(target_radius),
  armor_num_(armor_num),
  ekf_params_(ekf_params),
  p0_(p0)
{
  if (p0_.size() != 11) {
    p0_.resize(11);
    p0_ << 1, 64, 1, 64, 1, 64, 0.4, 400, 1.0e-5, 1, 1;
  }
}

auto_aim::Armor BaselineTracker::to_auto_aim_armor(const common::ArmorObservation & obs) const
{
  auto_aim::Armor a;
  a.name = auto_aim::ArmorName::one;
  a.type = auto_aim::ArmorType::small;
  a.priority = auto_aim::ArmorPriority::first;

  a.xyz_in_gimbal = obs.xyz_in_gimbal;
  a.xyz_in_world = obs.xyz_in_world;
  a.ypd_in_world = obs.ypd_in_world;  // 由 xyz_in_gimbal 得到
  a.yaw_raw = obs.yaw_raw;
  // EKF 工作系 = gimbal。第 4 维观测消费 ypr[0] = 装甲 yaw（已在 gimbal）。
  a.ypr_in_world = Eigen::Vector3d(obs.yaw_raw, 0.0, 0.0);
  a.confidence = obs.confidence;
  return a;
}

void BaselineTracker::init(const common::ArmorObservation & init_obs, double t)
{
  t0_ = std::chrono::steady_clock::now();
  current_t_ = t;
  auto armor = to_auto_aim_armor(init_obs);
  target_ = std::make_unique<auto_aim::Target>(
    armor, t0_, target_radius_, armor_num_, p0_, ekf_params_);
}

void BaselineTracker::predict(double dt)
{
  if (!target_) return;
  current_t_ += dt;
  target_->predict(dt);
}

void BaselineTracker::update(const common::ArmorObservation & obs)
{
  if (!target_) {
    init(obs, current_t_);
    return;
  }
  auto armor = to_auto_aim_armor(obs);
  target_->update(armor);
}

common::TargetEstimate BaselineTracker::estimate() const
{
  common::TargetEstimate est;
  if (!target_) return est;

  const Eigen::VectorXd & x = target_->ekf_x();
  est.x = x;
  if (x.size() >= 11) {
    est.center = Eigen::Vector3d(x[0], x[2], x[4]);
    est.velocity = Eigen::Vector3d(x[1], x[3], x[5]);
    est.yaw = x[6];
    est.w = x[7];
    est.radius = x[8];
    est.l = x[9];
    est.h = x[10];
    est.jumped = target_->jumped;
    est.armor_num = armor_num_;
    est.armor_xyza_list = target_->armor_xyza_list();
    est.last_nis = target_->ekf().last_nis;
    est.diverged = target_->diverged();
  }
  return est;
}

Eigen::Vector3d BaselineTracker::predict_aim_point(double flight_time) const
{
  if (!target_) return Eigen::Vector3d::Zero();

  const Eigen::VectorXd & x = target_->ekf_x();
  if (x.size() < 11) return Eigen::Vector3d::Zero();

  Eigen::Vector3d center(x[0], x[2], x[4]);
  Eigen::Vector3d vel(x[1], x[3], x[5]);
  double yaw = x[6];
  double r = x[8];

  // 当前估计的装甲板世界位置
  Eigen::Vector3d armor_pos = center + Eigen::Vector3d(-r * std::cos(yaw), -r * std::sin(yaw), 0.0);

  // 弹道前馈超前落点
  return armor_pos + vel * flight_time;
}

bool BaselineTracker::diverged() const
{
  return target_ ? target_->diverged() : false;
}

double BaselineTracker::last_nis() const
{
  return target_ ? target_->ekf().last_nis : 0.0;
}

}  // namespace tracker
