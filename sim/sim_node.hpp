#ifndef SIM__SIM_NODE_HPP
#define SIM__SIM_NODE_HPP

#include <atomic>
#include <cstdint>
#include <random>
#include <thread>

#include "channel.hpp"
#include "kinematics.hpp"
#include "types.hpp"

namespace sim
{

/// 仿真节点：固定 fps 推进物理并发布观测 FIFO + 真值快照。
/// 仿真时钟 sim_t = frame * dt；首帧 t=0。RNG 种子 42。
class SimNode
{
public:
  struct Topics
  {
    common::Channel<common::StampedObservation> & obs_out;
    common::LatestSlot<common::GroundTruth> & gt_out;
  };

  SimNode(const common::SimulationConfig & cfg, Topics topics);

  void start();
  void stop() { stop_requested_ = true; }
  void join();
  bool finished() const { return finished_.load(); }
  uint64_t frames_published() const { return frame_; }

private:
  void run();
  void publish(double sim_t, uint64_t frame);
  common::ArmorObservation observe(const common::GroundTruth & gt);

  common::SimulationConfig cfg_;
  Topics topics_;
  KinematicsEngine kinematics_;

  std::default_random_engine rng_{42};
  std::normal_distribution<double> dist_pos_;
  std::normal_distribution<double> dist_yaw_;

  std::thread thread_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> finished_{false};
  uint64_t frame_ = 0;
};

}  // namespace sim

#endif  // SIM__SIM_NODE_HPP
