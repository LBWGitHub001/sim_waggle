#ifndef SIM__SIM_NODE_HPP
#define SIM__SIM_NODE_HPP

#include <atomic>
#include <cstdint>
#include <thread>

#include "types.hpp"
#include "channel.hpp"
#include "kinematics.hpp"
#include "camera_sensor.hpp"

#include <omniscope/omniscope.hpp>

namespace sim
{

/// 仿真节点 (SimNode)
/// 类似 ROS Node 的独立线程：以固定频率 (fps) 推进物理仿真并发布三类"话题"：
/// - Channel<StampedObservation>       装甲板观测 FIFO（TrackerNode 订阅，不丢帧）
/// - LatestSlot<GroundTruth>           物理真值快照（可视化用）
/// - LatestSlot<StampedObservation>    最新观测快照（可视化画框用）
///
/// 仿真时钟为确定性时钟 sim_t = frame * dt，与墙钟无关；
/// 首帧发布 t=0 的初始观测（供 tracker 初始化，与旧单线程流程逐次对齐）。
class SimNode
{
public:
  // 节点对外发布的话题通道（由接线层创建，生命周期需长于本节点）
  struct Topics
  {
    common::Channel<common::StampedObservation> & obs_out;
    common::LatestSlot<common::GroundTruth> & gt_out;
    common::LatestSlot<common::StampedObservation> & obs_latest;
  };

  SimNode(const common::SimulationConfig & cfg, Topics topics);

  void start();                       // 启动节点线程
  void stop() { stop_requested_ = true; }  // 请求停止（跑完当前帧后退出）
  void join();                        // 等待线程退出
  bool finished() const { return finished_.load(); }  // duration 跑完自然结束
  uint64_t frames_published() const { return frame_; }  // join 后读取

private:
  void run();
  void publish(double sim_t, uint64_t frame);  // 发布一帧真值快照 + 观测

  common::SimulationConfig cfg_;
  Topics topics_;
  KinematicsEngine kinematics_;
  CameraSensor sensor_;

  std::thread thread_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> finished_{false};
  uint64_t frame_ = 0;  // 仅节点线程读写
};

}  // namespace sim

#endif  // SIM__SIM_NODE_HPP
