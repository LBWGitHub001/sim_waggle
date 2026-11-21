#ifndef TRACKER__TRACKER_NODE_HPP
#define TRACKER__TRACKER_NODE_HPP

#include <atomic>
#include <memory>
#include <thread>
#include <omniscope/omniscope.hpp>

#include "types.hpp"
#include "channel.hpp"
#include "metrics.hpp"
#include "baseline_tracker.hpp"

namespace tracker
{

/// 追踪节点：事件驱动消费观测 FIFO，predict+update，发布估计。
/// 误差分析与 kal_*/err_* / Kalman Marker 都在本节点、gimbal 工作系完成。
class TrackerNode
{
public:
  TrackerNode(
    std::unique_ptr<BaselineTracker> tracker,
    common::Channel<common::StampedObservation> & obs_in,
    common::LatestSlot<common::TrackerOutput> & est_out,
    common::LatestSlot<common::GroundTruth> & gt_in,
    double bullet_speed = 25.0);

  void start();
  void stop() { stop_requested_ = true; }
  void join();
  bool finished() const { return finished_.load(); }

private:
  void run();
  void publish_kalman_markers(
    const common::TargetEstimate & est, const Eigen::Vector3d & aim_lead_pt);

  std::unique_ptr<BaselineTracker> tracker_;
  common::Channel<common::StampedObservation> & obs_in_;
  common::LatestSlot<common::TrackerOutput> & est_out_;
  common::LatestSlot<common::GroundTruth> & gt_in_;
  double bullet_speed_ = 25.0;
  common::SimulationMetrics metrics_;

  std::thread thread_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> finished_{false};
};

}  // namespace tracker

#endif  // TRACKER__TRACKER_NODE_HPP
