#include "sim_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

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

  const auto obs = sensor_.observe(gt);
  common::StampedObservation msg{obs, sim_t, frame};
  topics_.obs_latest.set(msg);
  topics_.obs_out.push(std::move(msg));

  // ---------------- 遥测：观测组 + 真值组（本节点直接发布）----------------
  // 过渡：commit 3 会改成 gimbal 系 true_* / obs_*。此处先用 odom target 替代已删除的 target_inertial。
  const auto & enemy = gt.target;
  Eigen::Vector3d true_armor = Eigen::Vector3d::Zero();
  double true_armor_yaw = 0.0;
  const int visible_id = gt.visible_armor_id;
  if (visible_id >= 0 && visible_id < static_cast<int>(enemy.armors_pos.size())) {
    true_armor = enemy.armors_pos[visible_id];
    true_armor_yaw = enemy.armors_yaw[visible_id];
  }

  // 位置量先发（同量纲、可比）→ 看板默认勾选的就是它们
  omniscope::plot({
    {"obs_armor_x", obs.xyz_in_world.x()},  // 可见装甲板 x (m) 世界系
    {"obs_armor_y", obs.xyz_in_world.y()},
    {"obs_armor_z", obs.xyz_in_world.z()},
  });
  omniscope::plot({
    {"true_center_x", enemy.center.x()}, // 敌车中心 x (m) 惯性系
    {"true_center_y", enemy.center.y()},
    {"true_center_z", enemy.center.z()},
  });

  // 其余量（角度/角速度/线速度/装甲板真值）后发 → 侧栏按需勾选
  omniscope::plot({
    {"obs_armor_yaw_deg", obs.yaw_raw * 180.0 / M_PI},        // 装甲板 yaw   (deg)
    {"obs_w_ego", gt.chassis.w},                              // 本车自转     (rad/s)
    {"true_armor_x", true_armor.x()},                         // 可见装甲板 x (m) 惯性系
    {"true_armor_y", true_armor.y()},
    {"true_armor_z", true_armor.z()},
    {"true_armor_yaw_deg", true_armor_yaw * 180.0 / M_PI},    // 装甲板 yaw   (deg)
    {"true_vel_x", enemy.vel.x()},                           // 敌车速度 x   (m/s) 惯性系
    {"true_vel_y", enemy.vel.y()},
    {"true_vel_z", enemy.vel.z()},
    {"true_yaw_deg", enemy.yaw * 180.0 / M_PI},               // 目标朝向 yaw (deg)
    {"true_w", enemy.w},                                      // 目标自转 w   (rad/s)
    {"true_w_ego", gt.chassis.w},                             // 本车自转真值 (rad/s)
  });
}

}  // namespace sim
