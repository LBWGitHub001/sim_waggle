#ifndef TRACKER__EGO_CONTROL_TRACKER_HPP
#define TRACKER__EGO_CONTROL_TRACKER_HPP

#include <memory>
#include <string>
#include "types.hpp"
#include "target.hpp"

namespace tracker
{

/// 把云台系 xy 加速度作为控制量 u 注入过程模型：x = Fx + Bu。
/// 不做圆周/杆臂建模；z 不加控制。
class EgoControlTracker
{
public:
  EgoControlTracker(
    double target_radius = 0.20,
    int armor_num = 4,
    const auto_aim::EkfParams & ekf_params = auto_aim::EkfParams{},
    const Eigen::VectorXd & p0 = Eigen::VectorXd::Zero(11));

  void set_control(const Eigen::Vector3d & ego_acc);
  void init(const common::ArmorObservation & init_obs, double t);
  void predict(double dt);
  void update(const common::ArmorObservation & obs);

  common::TargetEstimate estimate() const;
  Eigen::Vector3d predict_aim_point(double flight_time) const;
  bool diverged() const;
  double last_nis() const;
  std::string name() const { return "EgoControlTracker"; }

private:
  auto_aim::Armor to_auto_aim_armor(const common::ArmorObservation & obs) const;

  double target_radius_;
  int armor_num_;
  auto_aim::EkfParams ekf_params_;
  Eigen::VectorXd p0_;
  Eigen::Vector3d u_ = Eigen::Vector3d::Zero();

  std::unique_ptr<auto_aim::Target> target_;
  std::chrono::steady_clock::time_point t0_;
  double current_t_ = 0.0;
};

}  // namespace tracker

#endif  // TRACKER__EGO_CONTROL_TRACKER_HPP
