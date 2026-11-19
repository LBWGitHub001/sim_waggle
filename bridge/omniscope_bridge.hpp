#ifndef BRIDGE__OMNISCOPE_BRIDGE_HPP
#define BRIDGE__OMNISCOPE_BRIDGE_HPP

#include <cstdint>
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

  // 发布遥测：误差曲线（键名与 MetricsResult 字段一致）+ 真值/观测值原始量
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
};

}  // namespace bridge

#endif  // BRIDGE__OMNISCOPE_BRIDGE_HPP
