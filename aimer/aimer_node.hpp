#ifndef AIMER__AIMER_NODE_HPP
#define AIMER__AIMER_NODE_HPP

#include <atomic>
#include <thread>

#include "aim.hpp"
#include "channel.hpp"
#include "types.hpp"

namespace aimer
{

/// 火控节点：同一套 Aimer::aim(state) 分别吃 Kalman 与 GT，得到估计 cmd 与真值 cmd。
class AimerNode
{
public:
  AimerNode(
    common::AimerParams aimer,
    common::Channel<common::TrackerOutput> & est_in,
    common::LatestSlot<common::GroundTruth> & gt_in,
    common::LatestSlot<common::GimbalCommand> & cmd_out,
    double bullet_speed = 25.0);

  void start();
  void stop() { stop_requested_ = true; }
  void join();
  bool finished() const { return finished_.load(); }

private:
  void run();
  common::GimbalCommand cmd_from_state(common::Aimer & aimer, const common::TargetEstimate & est);
  void publish_aim_markers(const Eigen::Vector3d & aim_lead_pt, uint64_t stamp_ns);

  common::Aimer aimer_est_;
  common::Aimer aimer_gt_;
  common::Channel<common::TrackerOutput> & est_in_;
  common::LatestSlot<common::GroundTruth> & gt_in_;
  common::LatestSlot<common::GimbalCommand> & cmd_out_;
  double bullet_speed_ = 25.0;

  std::thread thread_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> finished_{false};
};

}  // namespace aimer

#endif  // AIMER__AIMER_NODE_HPP
