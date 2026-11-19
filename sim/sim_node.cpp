#include "sim_node.hpp"

#include <algorithm>
#include <chrono>

namespace sim
{

SimNode::SimNode(const common::SimulationConfig & cfg, Topics topics)
: cfg_(cfg), topics_(topics), kinematics_(cfg), sensor_(cfg)
{
}

void SimNode::start()
{
  thread_ = std::thread(&SimNode::run, this);
}

void SimNode::join()
{
  if (thread_.joinable()) thread_.join();
}

void SimNode::run()
{
  const int fps = std::max(cfg_.fps, 1);
  const double dt = 1.0 / static_cast<double>(fps);
  const auto frame_interval = std::chrono::microseconds(1000000 / fps);

  // t = 0 初始帧：发布初始真值与初始观测（TrackerNode 用首帧做 init）
  publish(0.0, 0);

  auto next_tick = std::chrono::steady_clock::now();
  while (!stop_requested_.load()) {
    // 1. 物理推进一帧
    kinematics_.step(dt);
    frame_++;
    const double sim_t = frame_ * dt;  // 确定性仿真时钟

    // 2. 发布真值快照与观测
    publish(sim_t, frame_);

    // duration 到点：发完本帧后收尾
    if (cfg_.duration > 0.0 && sim_t >= cfg_.duration) break;

    // 3. 维持固定帧率
    next_tick += frame_interval;
    std::this_thread::sleep_until(next_tick);
  }

  // 4. 关闭观测通道：下游 TrackerNode 排空后退出
  topics_.obs_out.close();
  finished_ = true;
}

void SimNode::publish(double sim_t, uint64_t frame)
{
  const auto & gt = kinematics_.ground_truth();
  topics_.gt_out.set(gt);

  common::StampedObservation msg{sensor_.observe(gt), sim_t, frame};
  topics_.obs_latest.set(msg);
  topics_.obs_out.push(std::move(msg));
}

}  // namespace sim
