#ifndef TRACKER__TARGET_HPP
#define TRACKER__TARGET_HPP

#include <Eigen/Dense>

#include "extended_kalman_filter.hpp"
#include "types.hpp"

namespace tracker
{

struct EkfParams
{
  double q_xyz_acc_var = 100.0;
  double q_yaw_acc_var = 75.0;
  double m_ax = 1.0;
  double m_ay = 1.0;
  double r_yaw = 0.04;
  double r_pitch = 0.04;
  double r_dist_base = 0.015;
};

/// 11 维 CV + 常角速度 EKF。观测 id 来自仿真，不做切板匹配。
/// 调用序：predict(dt) → control(ego_acc) → update(obs)。
class Target
{
public:
  Target() = default;
  Target(double radius, int armor_num, Eigen::VectorXd P0_dig, EkfParams ekf_params);

  void init(const common::ArmorObservation & obs);
  void predict(double dt);
  void control(const Eigen::Vector3d & acc);
  void update(const common::ArmorObservation & obs);

  common::TargetEstimate estimate() const;
  bool initialized() const { return initialized_; }

private:
  void update_ypda(const common::ArmorObservation & obs, int id);

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;

  int armor_id(const common::ArmorObservation & obs) const;

  int armor_num_ = 4;
  double radius_ = 0.20;
  Eigen::VectorXd p0_dig_;
  EkfParams ekf_params_;
  tools::ExtendedKalmanFilter ekf_;
  bool initialized_ = false;
  double last_dt_ = 0.0;
};

}  // namespace tracker

#endif  // TRACKER__TARGET_HPP
