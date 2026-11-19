#include "tracker_node.hpp"

#include <cmath>

#include "fft.hpp"

namespace tracker
{

TrackerNode::TrackerNode(
  std::unique_ptr<ITracker> tracker,
  common::Channel<common::StampedObservation> & obs_in,
  common::LatestSlot<common::TrackerOutput> & est_out)
: tracker_(std::move(tracker)), obs_in_(obs_in), est_out_(est_out)
{
}

void TrackerNode::start()
{
  thread_ = std::thread(&TrackerNode::run, this);
}

void TrackerNode::join()
{
  if (thread_.joinable()) thread_.join();
}

void TrackerNode::run()
{
  common::StampedObservation msg;
  double last_t = -1.0;

  // 频域发布演示：对观测与估计速度各挂一个滑窗 FFT（N=512, hop=256）
  // 预期：obs_x 频谱在小陀螺频率（12 rad/s ≈ 1.91 Hz）处出现明显谱峰

  while (!stop_requested_.load() && obs_in_.wait_pop(msg)) {
    if (last_t < 0.0) {
      // 首帧观测：直接 update（ITracker 实现内部自动 init，等价旧流程 init(obs, 0.0)）
      tracker_->update(msg.obs);
    } else {
      // 后续帧：predict(距上帧的仿真时长) + update
      tracker_->predict(msg.sim_t - last_t);
      tracker_->update(msg.obs);
    }
    last_t = msg.sim_t;

    auto target = tracker_->estimate();
    // 发布估计快照（可视化线程读取）
    est_out_.set(common::TrackerOutput{target, msg.sim_t});

    // ---------------- 遥测：Kalman 状态组（本节点直接发布）----------------
    // 坐标系：云台中心（与观测 xyz_in_world 同一系）
    if (target.x.size() >= 11) {
      omniscope::plot({
        {"kal_center_x", target.center.x()},        // 目标中心 x   (m)
        {"kal_center_y", target.center.y()},        // 目标中心 y   (m)
        {"kal_center_z", target.center.z()},        // 目标中心 z   (m)
        {"kal_vel_x", target.velocity.x()},         // 目标速度 x   (m/s)
        {"kal_vel_y", target.velocity.y()},         // 目标速度 y   (m/s)
        {"kal_vel_z", target.velocity.z()},         // 目标速度 z   (m/s)
        {"kal_yaw_deg", target.yaw * 180.0 / M_PI}, // 目标朝向 yaw (deg)
        {"kal_w", target.w},                        // 目标自转 w   (rad/s)
        {"kal_radius_mm", target.radius * 1000.0},  // 装甲板半径   (mm)
      });
    }
  }

  finished_ = true;
}

}  // namespace tracker
