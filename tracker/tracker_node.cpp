#include "tracker_node.hpp"

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

  }

  finished_ = true;
}

}  // namespace tracker
