#include "ego_control_tracker.hpp"

#include <cmath>

namespace tracker
{

EgoControlTracker::EgoControlTracker(
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

void EgoControlTracker::control(const Eigen::Vector3d & ego_acc)
{
  if (!target_) return;
  target_->control(ego_acc);
}

auto_aim::Armor EgoControlTracker::to_auto_aim_armor(const common::ArmorObservation & obs) const
{
  auto_aim::Armor a;
  a.name = auto_aim::ArmorName::one;
  a.type = auto_aim::ArmorType::small;
  a.priority = auto_aim::ArmorPriority::first;

  a.xyz_in_gimbal = obs.xyz_in_gimbal;
  a.xyz_in_world = obs.xyz_in_world;
  a.ypd_in_world = obs.ypd_in_world;
  a.yaw_raw = obs.yaw_raw;
  a.ypr_in_world = Eigen::Vector3d(obs.yaw_raw, 0.0, 0.0);
  a.confidence = obs.confidence;
  return a;
}

void EgoControlTracker::init(const common::ArmorObservation & init_obs, double t)
{
  t0_ = std::chrono::steady_clock::now();
  current_t_ = t;
  auto armor = to_auto_aim_armor(init_obs);
  target_ = std::make_unique<auto_aim::Target>(
    armor, t0_, target_radius_, armor_num_, p0_, ekf_params_);
}

void EgoControlTracker::predict(double dt)
{
  if (!target_) return;
  current_t_ += dt;
  target_->predict(dt);
}

void EgoControlTracker::update(const common::ArmorObservation & obs)
{
  if (!target_) {
    init(obs, current_t_);
    return;
  }
  auto armor = to_auto_aim_armor(obs);
  target_->update(armor);
}

common::TargetEstimate EgoControlTracker::estimate() const
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
    est.armor_xyza_list = target_->armor_xyza_list();
    est.last_nis = target_->ekf().last_nis;
    est.diverged = target_->diverged();
  }
  return est;
}

Eigen::Vector3d EgoControlTracker::predict_aim_point(double flight_time) const
{
  if (!target_) return Eigen::Vector3d::Zero();

  const Eigen::VectorXd & x = target_->ekf_x();
  if (x.size() < 11) return Eigen::Vector3d::Zero();

  Eigen::Vector3d center(x[0], x[2], x[4]);
  Eigen::Vector3d vel(x[1], x[3], x[5]);
  double yaw = x[6];
  double r = x[8];

  Eigen::Vector3d armor_pos = center + Eigen::Vector3d(-r * std::cos(yaw), -r * std::sin(yaw), 0.0);
  return armor_pos + vel * flight_time;
}

bool EgoControlTracker::diverged() const
{
  return target_ ? target_->diverged() : false;
}

double EgoControlTracker::last_nis() const
{
  return target_ ? target_->ekf().last_nis : 0.0;
}

}  // namespace tracker
