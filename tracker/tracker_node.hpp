#ifndef TRACKER__TRACKER_NODE_HPP
#define TRACKER__TRACKER_NODE_HPP

#include <atomic>
#include <thread>

#include "channel.hpp"
#include "metrics.hpp"
#include "target.hpp"
#include "types.hpp"

namespace tracker
{

/// 追踪节点：消费观测 FIFO，直接持有 Target（EKF），发布估计。不丢帧。
class TrackerNode
{
public:
  TrackerNode(
    double target_radius,
    int armor_num,
    EkfParams ekf,
    Eigen::VectorXd p0,
    common::Channel<common::StampedObservation> & obs_in,
    common::Channel<common::TrackerOutput> & est_out,
    common::LatestSlot<common::GroundTruth> & gt_in,
    double bullet_speed = 25.0);

  void start();
  void stop() { stop_requested_ = true; }
  void join();
  bool finished() const { return finished_.load(); }

private:
  void run();
  void publish_kalman_markers(const common::TargetEstimate & est);

  Target target_;
  common::Channel<common::StampedObservation> & obs_in_;
  common::Channel<common::TrackerOutput> & est_out_;
  common::LatestSlot<common::GroundTruth> & gt_in_;
  double bullet_speed_ = 25.0;
  common::SimulationMetrics metrics_;

  std::thread thread_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> finished_{false};
};

}  // namespace tracker

#endif  // TRACKER__TRACKER_NODE_HPP
