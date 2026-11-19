#ifndef BRIDGE__OMNISCOPE_BRIDGE_HPP
#define BRIDGE__OMNISCOPE_BRIDGE_HPP

#include <cstdint>
#include <deque>
#include <string>
#include <vector>
#include <utility>
#include <fmt/format.h>
#include <opencv2/core.hpp>

#include "common/types.hpp"
#include "common/metrics.hpp"

namespace bridge
{

struct BridgeConfig
{
  uint16_t control_port = 15000;
  uint16_t beacon_port = 15999;
  bool enable_remote = true;
  bool enable_local = true;
  std::string log_dir = "./logs";
  std::string sender_name = "sim_waggle";
  int img_width = 800;
  int img_quality = 60;
  uint32_t heartbeat_interval_ms = 500;
  uint32_t beacon_interval_ms = 1000;
};

/// OmniScope 隔离适配器 (Bridge / Adapter Pattern)
/// 零侵入封装 OmniScope SDK，向外提供清晰纯粹的领域层 API：
/// - 遥测数据打包与高频 UDP 广播
/// - 标准 ROS 2 3D Marker 生成与下发
/// - 车载相机检测图像压缩推送
class OmniScopeBridge
{
public:
  explicit OmniScopeBridge(const BridgeConfig & cfg = BridgeConfig{});
  ~OmniScopeBridge();

  // 发布估计误差遥测：
  //   核心 6 条（pos_err_norm / vel_err_norm / rel_pos_err / rel_vel_err /
  //   lead_bias_mm / nis）先单独成包，使 OmniScope 看板默认勾选的就是它们；
  //   明细曲线（分轴误差、yaw、装甲板拟合、真值/估计速度对比、累计 RMS）随后发布；
  //   另以 5Hz 刷新云台系 x-y 误差轨迹 err_locus。
  void publish_telemetry(
    const common::GroundTruth & gt,
    const common::ArmorObservation & obs,
    const common::TargetEstimate & est,
    const common::MetricsResult & metrics);

  // 发布 ROS 2 3D Markers (包含 8 大命名空间: 底盘、云台、真值目标/装甲、EKF估计/伪速度、前馈瞄准)
  void publish_3d_markers(
    const common::GroundTruth & gt,
    const common::TargetEstimate & est,
    const Eigen::Vector3d & aim_lead_pt);

  // 发布车载相机检测图像
  void publish_camera_image(
    const cv::Mat & image,
    const std::string & stream_name = "camera_detector");

  // 远程与本地结构化日志
  void log(const std::string & level, const std::string & msg);

  template <typename... Args>
  void log(const std::string & level, const std::string & fmt_str, Args &&... args)
  {
    log(level, fmt::format(fmt::runtime(fmt_str), std::forward<Args>(args)...));
  }

  // 释放资源
  void shutdown();

private:
  BridgeConfig cfg_;

  // 误差轨迹（云台系 x-y）环形缓冲，用于 XY 曲线 err_locus
  std::deque<double> locus_x_;
  std::deque<double> locus_y_;
  uint32_t pub_frames_ = 0;
};

}  // namespace bridge

#endif  // BRIDGE__OMNISCOPE_BRIDGE_HPP
