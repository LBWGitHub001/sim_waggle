#ifndef TRACKER__TRACKER_NODE_HPP
#define TRACKER__TRACKER_NODE_HPP

#include <atomic>
#include <memory>
#include <thread>
#include <omniscope/omniscope.hpp>

#include "types.hpp"
#include "channel.hpp"
#include "baseline_tracker.hpp"

namespace tracker
{

/// 追踪节点 (TrackerNode)
/// 类似 ROS Node 的独立线程：事件驱动订阅装甲板观测 FIFO 通道，
/// 逐条执行 predict + update，并发布最新状态估计快照。
/// 每条观测都被处理（不丢帧、保序），tracker 看到的调用序列
/// 与旧单线程流水线完全一致（首帧 init，之后每帧 predict(dt)+update）。
class TrackerNode
{
public:
  // tracker: 由接线层构造注入（本分支具体类型 BaselineTracker）；
  // obs_in / est_out: 话题通道，生命周期需长于本节点
  TrackerNode(
    std::unique_ptr<BaselineTracker> tracker,
    common::Channel<common::StampedObservation> & obs_in,
    common::LatestSlot<common::TrackerOutput> & est_out);

  void start();                       // 启动节点线程
  void stop() { stop_requested_ = true; }  // 请求停止（处理完当前一条后退出）
  void join();                        // 等待线程退出
  bool finished() const { return finished_.load(); }

private:
  void run();

  std::unique_ptr<BaselineTracker> tracker_;
  common::Channel<common::StampedObservation> & obs_in_;
  common::LatestSlot<common::TrackerOutput> & est_out_;

  std::thread thread_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> finished_{false};
};

}  // namespace tracker

#endif  // TRACKER__TRACKER_NODE_HPP
