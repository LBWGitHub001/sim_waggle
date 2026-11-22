#ifndef AIMER__AIMER_NODE_HPP
#define AIMER__AIMER_NODE_HPP

#include <atomic>
#include <thread>

#include "aim.hpp"
#include "channel.hpp"
#include "types.hpp"

namespace aimer
{

/// 火控节点：消费 TrackerOutput FIFO，Aimer::aim 出无约束射击角，发 cmd_* / aim.lead。
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
  void publish_aim_markers(const Eigen::Vector3d & aim_lead_pt);

  common::Aimer aimer_;
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
