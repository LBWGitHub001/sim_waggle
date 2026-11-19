#ifndef TRACKER__BASELINE_TRACKER_HPP
#define TRACKER__BASELINE_TRACKER_HPP

#include <memory>
#include "itracker.hpp"
#include "target.hpp"
#include "extended_kalman_filter.hpp"

namespace tracker
{

/// 基线卡尔曼追踪器 (BaselineTracker)
/// 封装 auto_aim 原版 Target 与 11 维 EKF 模型，
/// 保持原算法逻辑 100% 忠实还原。
class BaselineTracker : public ITracker
{
public:
  BaselineTracker(
    double target_radius = 0.20,
    int armor_num = 4,
    const auto_aim::EkfParams & ekf_params = auto_aim::EkfParams{},
    const Eigen::VectorXd & p0 = Eigen::VectorXd::Zero(11));

  void init(const common::ArmorObservation & init_obs, double t) override;
  void predict(double dt) override;
  void update(const common::ArmorObservation & obs) override;

  common::TargetEstimate estimate() const override;
  Eigen::Vector3d predict_aim_point(double flight_time) const override;
  bool diverged() const override;
  double last_nis() const override;
  std::string name() const override { return "BaselineTracker"; }

private:
  auto_aim::Armor to_auto_aim_armor(const common::ArmorObservation & obs) const;

  double target_radius_;
  int armor_num_;
  auto_aim::EkfParams ekf_params_;
  Eigen::VectorXd p0_;

  std::unique_ptr<auto_aim::Target> target_;
  std::chrono::steady_clock::time_point t0_;
  double current_t_ = 0.0;
};

}  // namespace tracker

#endif  // TRACKER__BASELINE_TRACKER_HPP
