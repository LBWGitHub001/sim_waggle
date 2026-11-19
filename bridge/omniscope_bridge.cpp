#include "omniscope_bridge.hpp"

#include <omniscope/omniscope.hpp>
#include <omniscope/remote_logger.hpp>
#include <omniscope/marker.hpp>
#include <nlohmann/json.hpp>
#include <fmt/format.h>
#include <cmath>

namespace bridge
{
namespace
{
constexpr double LIGHTBAR_LENGTH = 56e-3;
constexpr double SMALL_ARMOR_WIDTH = 135e-3;
constexpr double ARMOR_THICKNESS = 0.015;

omniscope::Quaternion eigen_quat_to_omni(const Eigen::Quaterniond & q)
{
  omniscope::Quaternion oq;
  oq.x = static_cast<float>(q.x());
  oq.y = static_cast<float>(q.y());
  oq.z = static_cast<float>(q.z());
  oq.w = static_cast<float>(q.w());
  return oq;
}

Eigen::Quaterniond armor_quat(double yaw)
{
  const double pitch = 15.0 * CV_PI / 180.0;
  const double sy = std::sin(yaw);
  const double cy = std::cos(yaw);
  const double sp = std::sin(pitch);
  const double cp = std::cos(pitch);

  Eigen::Matrix3d R;
  R << cy * cp, -sy, cy * sp,
       sy * cp,  cy, sy * sp,
           -sp,   0,      cp;
  return Eigen::Quaterniond(R);
}

omniscope::Vector3 to_vec3(const Eigen::Vector3d & v)
{
  return {static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())};
}

omniscope::ColorRGBA to_rgba(float r, float g, float b, float a = 1.0f)
{
  return {r, g, b, a};
}
}  // namespace

OmniScopeBridge::OmniScopeBridge(const BridgeConfig & cfg)
: cfg_(cfg)
{
  omniscope::RemoteLogger::Config rc;
  rc.control_port = cfg_.control_port;
  rc.beacon_port = cfg_.beacon_port;
  rc.enable_remote = cfg_.enable_remote;
  rc.enable_local = cfg_.enable_local;
  rc.log_dir = cfg_.log_dir;
  rc.sender_name = cfg_.sender_name;
  rc.heartbeat_interval_ms = cfg_.heartbeat_interval_ms;
  rc.beacon_interval_ms = cfg_.beacon_interval_ms;
  rc.img_width = cfg_.img_width;
  rc.img_quality = cfg_.img_quality;

  omniscope::RemoteLogger::instance().init(rc);
}

OmniScopeBridge::~OmniScopeBridge()
{
  shutdown();
}

void OmniScopeBridge::shutdown()
{
  omniscope::RemoteLogger::instance().shutdown();
}

void OmniScopeBridge::log(const std::string & level, const std::string & msg)
{
  omniscope::RemoteLogger::instance().log(level, msg);
}

void OmniScopeBridge::publish_camera_image(
  const cv::Mat & image, const std::string & stream_name)
{
  omniscope::RemoteLogger::instance().plot_image(image, {{"name", stream_name}});
}

void OmniScopeBridge::publish_telemetry(
  const common::GroundTruth & gt,
  const common::ArmorObservation & obs,
  const common::TargetEstimate & est,
  const common::MetricsResult & metrics)
{
  // 估计尚未成形时不上报，避免 0 值污染曲线
  if (est.x.size() < 11) return;

  // 1) 误差曲线（先发 → 看板默认勾选这 6 条）
  //    键名与 MetricsResult 字段一一对应
  omniscope::plot({
    {"err_x", metrics.err_x},               // 位置误差 x        (m)
    {"err_y", metrics.err_y},               // 位置误差 y        (m)
    {"pos_err_norm", metrics.pos_err_norm}, // 位置误差模长      (m)
    {"err_vx", metrics.err_vx},             // 速度误差 x        (m/s)
    {"err_vy", metrics.err_vy},             // 速度误差 y        (m/s)
    {"vel_err_norm", metrics.vel_err_norm}, // 速度误差模长/伪速度 (m/s)
  });

  // 2) 真值 / 观测值 / Kalman 估计值（后发 → 侧栏按需勾选）
  //    三者统一在"世界系原点位于云台"的坐标系下：该系随云台平移但不随其旋转，
  //    与观测 xyz_in_world = armor_pos - gimbal.pos 完全一致。
  //    因此真值里混有云台偏置公转的运动（v_true = v_target - v_gimbal）。
  omniscope::plot({
    {"true_x", gt.target.center.x() - gt.gimbal.pos.x()},  // 目标相对云台的 x   (m)
    {"true_y", gt.target.center.y() - gt.gimbal.pos.y()},  // 目标相对云台的 y   (m)
    {"true_vx", gt.target.vel.x() - gt.gimbal.vel.x()},    // 相对速度 x         (m/s)
    {"true_vy", gt.target.vel.y() - gt.gimbal.vel.y()},    // 相对速度 y         (m/s)
    {"obs_x", obs.xyz_in_world.x()},                       // 可见装甲板观测 x   (m)
    {"obs_y", obs.xyz_in_world.y()},                       // 可见装甲板观测 y   (m)
    {"kalman_x", est.center.x()},                          // Kalman 估计中心 x  (m)
    {"kalman_y", est.center.y()},                          // Kalman 估计中心 y  (m)
    {"kalman_vx", est.velocity.x()},                       // Kalman 估计速度 x  (m/s)
    {"kalman_vy", est.velocity.y()},                       // Kalman 估计速度 y  (m/s)
  });
}

void OmniScopeBridge::publish_3d_markers(
  const common::GroundTruth & gt,
  const common::TargetEstimate & est,
  const Eigen::Vector3d & aim_lead_pt)
{
  std::vector<omniscope::Marker> markers;
  markers.reserve(20);

  const std::string frame = "world";

  // 1. 底盘 (my_robot.chassis)
  {
    omniscope::Marker m;
    m.frame_id = frame;
    m.ns = "my_robot.chassis";
    m.id = 0;
    m.type = omniscope::MarkerType::SPHERE;
    m.action = omniscope::MarkerAction::ADD;
    m.pose.position = to_vec3(gt.chassis.pos);
    m.scale = {0.16f, 0.16f, 0.16f};
    m.color = to_rgba(0.0f, 0.94f, 1.0f, 0.95f);
    markers.push_back(m);
  }
  {
    omniscope::Marker m;
    m.frame_id = frame;
    m.ns = "my_robot.chassis";
    m.id = 1;
    m.type = omniscope::MarkerType::ARROW;
    m.action = omniscope::MarkerAction::ADD;
    Eigen::Vector3d end_pt = gt.chassis.pos + Eigen::Vector3d(0.40 * std::cos(gt.chassis.yaw), 0.40 * std::sin(gt.chassis.yaw), 0.0);
    m.points = {to_vec3(gt.chassis.pos), to_vec3(end_pt)};
    m.scale = {0.035f, 0.07f, 0.07f};
    m.color = to_rgba(0.0f, 0.94f, 1.0f, 1.0f);
    markers.push_back(m);
  }
  // 2. 云台与偏置连杆 (my_robot.gimbal)
  {
    omniscope::Marker m;
    m.frame_id = frame;
    m.ns = "my_robot.gimbal";
    m.id = 10;
    m.type = omniscope::MarkerType::LINE_LIST;
    m.action = omniscope::MarkerAction::ADD;
    m.points = {to_vec3(gt.chassis.pos), to_vec3(gt.gimbal.pos)};
    m.scale = {0.015f, 0.0f, 0.0f};
    m.color = to_rgba(1.0f, 0.85f, 0.1f, 1.0f);
    markers.push_back(m);
  }
  {
    omniscope::Marker m;
    m.frame_id = frame;
    m.ns = "my_robot.gimbal";
    m.id = 11;
    m.type = omniscope::MarkerType::SPHERE;
    m.action = omniscope::MarkerAction::ADD;
    m.pose.position = to_vec3(gt.gimbal.pos);
    m.scale = {0.11f, 0.11f, 0.11f};
    m.color = to_rgba(1.0f, 0.85f, 0.1f, 1.0f);
    markers.push_back(m);
  }
  {
    omniscope::Marker m;
    m.frame_id = frame;
    m.ns = "my_robot.gimbal";
    m.id = 12;
    m.type = omniscope::MarkerType::ARROW;
    m.action = omniscope::MarkerAction::ADD;
    Eigen::Vector3d aim_dir = gt.gimbal.pos + Eigen::Vector3d(0.50 * std::cos(gt.gimbal.yaw), 0.50 * std::sin(gt.gimbal.yaw), 0.0);
    m.points = {to_vec3(gt.gimbal.pos), to_vec3(aim_dir)};
    m.scale = {0.025f, 0.05f, 0.05f};
    m.color = to_rgba(1.0f, 0.95f, 0.2f, 1.0f);
    markers.push_back(m);
  }
  // 3. 真实物理目标 (ground_truth)
  {
    omniscope::Marker m;
    m.frame_id = frame;
    m.ns = "ground_truth.target";
    m.id = 20;
    m.type = omniscope::MarkerType::SPHERE;
    m.action = omniscope::MarkerAction::ADD;
    m.pose.position = to_vec3(gt.target.center);
    m.scale = {0.11f, 0.11f, 0.11f};
    m.color = to_rgba(0.1f, 1.0f, 0.3f, 0.95f);
    markers.push_back(m);
  }
  std::vector<omniscope::Vector3> gt_spokes;
  for (size_t i = 0; i < gt.target.armors_pos.size(); i++) {
    omniscope::Marker m;
    m.frame_id = frame;
    m.ns = "ground_truth.armor";
    m.id = static_cast<int32_t>(30 + i);
    m.type = omniscope::MarkerType::CUBE;
    m.action = omniscope::MarkerAction::ADD;
    m.pose.position = to_vec3(gt.target.armors_pos[i]);
    m.pose.orientation = eigen_quat_to_omni(armor_quat(gt.target.armors_yaw[i]));
    m.scale = {static_cast<float>(ARMOR_THICKNESS), static_cast<float>(SMALL_ARMOR_WIDTH), static_cast<float>(LIGHTBAR_LENGTH)};
    m.color = to_rgba(0.1f, 0.95f, 0.3f, 0.9f);
    markers.push_back(m);

    gt_spokes.push_back(to_vec3(gt.target.center));
    gt_spokes.push_back(to_vec3(gt.target.armors_pos[i]));
  }
  {
    omniscope::Marker m;
    m.frame_id = frame;
    m.ns = "ground_truth.armor";
    m.id = 40;
    m.type = omniscope::MarkerType::LINE_LIST;
    m.action = omniscope::MarkerAction::ADD;
    m.points = gt_spokes;
    m.scale = {0.005f, 0.0f, 0.0f};
    m.color = to_rgba(0.2f, 0.7f, 0.3f, 0.6f);
    markers.push_back(m);
  }

  // 4. 自瞄 EKF 估计目标 (kalman)
  if (est.x.size() >= 11) {
    Eigen::Vector3d ekf_center(est.center.x(), est.center.y(), 0.0);
    Eigen::Vector3d ekf_vel(est.velocity.x(), est.velocity.y(), 0.0);

    {
      omniscope::Marker m;
      m.frame_id = frame;
      m.ns = "kalman.center";
      m.id = 50;
      m.type = omniscope::MarkerType::SPHERE;
      m.action = omniscope::MarkerAction::ADD;
      m.pose.position = to_vec3(ekf_center);
      m.scale = {0.10f, 0.10f, 0.10f};
      m.color = to_rgba(1.0f, 0.45f, 0.05f, 0.95f);
      markers.push_back(m);
    }
    if (ekf_vel.norm() > 0.05) {
      omniscope::Marker m;
      m.frame_id = frame;
      m.ns = "kalman.vel";
      m.id = 52;
      m.type = omniscope::MarkerType::ARROW;
      m.action = omniscope::MarkerAction::ADD;
      Eigen::Vector3d v_end = ekf_center + ekf_vel * 0.35;
      m.points = {to_vec3(ekf_center), to_vec3(v_end)};
      m.scale = {0.035f, 0.07f, 0.07f};
      m.color = to_rgba(1.0f, 0.25f, 0.05f, 1.0f);
      markers.push_back(m);
    }

    std::vector<omniscope::Vector3> ekf_spokes;
    for (size_t i = 0; i < est.armor_xyza_list.size(); i++) {
      Eigen::Vector3d p_plate(est.armor_xyza_list[i][0], est.armor_xyza_list[i][1], est.armor_xyza_list[i][2]);
      double yaw_plate = est.armor_xyza_list[i][3];

      omniscope::Marker m;
      m.frame_id = frame;
      m.ns = "kalman.armor";
      m.id = static_cast<int32_t>(60 + i);
      m.type = omniscope::MarkerType::CUBE;
      m.action = omniscope::MarkerAction::ADD;
      m.pose.position = to_vec3(p_plate);
      m.pose.orientation = eigen_quat_to_omni(armor_quat(yaw_plate));
      m.scale = {static_cast<float>(ARMOR_THICKNESS), static_cast<float>(SMALL_ARMOR_WIDTH), static_cast<float>(LIGHTBAR_LENGTH)};
      m.color = to_rgba(0.15f, 0.60f, 1.0f, 0.90f);
      markers.push_back(m);

      ekf_spokes.push_back(to_vec3(ekf_center));
      ekf_spokes.push_back(to_vec3(p_plate));
    }
    {
      omniscope::Marker m;
      m.frame_id = frame;
      m.ns = "kalman.armor";
      m.id = 70;
      m.type = omniscope::MarkerType::LINE_LIST;
      m.action = omniscope::MarkerAction::ADD;
      m.points = ekf_spokes;
      m.scale = {0.005f, 0.0f, 0.0f};
      m.color = to_rgba(0.25f, 0.55f, 0.9f, 0.65f);
      markers.push_back(m);
    }
  }

  // 5. 弹道前馈超前瞄准点 (aim.lead)
  {
    omniscope::Marker m;
    m.frame_id = frame;
    m.ns = "aim.lead";
    m.id = 80;
    m.type = omniscope::MarkerType::SPHERE;
    m.action = omniscope::MarkerAction::ADD;
    m.pose.position = to_vec3(aim_lead_pt);
    m.scale = {0.09f, 0.09f, 0.09f};
    m.color = to_rgba(1.0f, 0.85f, 0.0f, 1.0f);
    markers.push_back(m);
  }
  {
    omniscope::Marker m;
    m.frame_id = frame;
    m.ns = "aim.lead";
    m.id = 82;
    m.type = omniscope::MarkerType::LINE_LIST;
    m.action = omniscope::MarkerAction::ADD;
    m.points = {to_vec3(gt.gimbal.pos), to_vec3(aim_lead_pt)};
    m.scale = {0.008f, 0.0f, 0.0f};
    m.color = to_rgba(1.0f, 0.8f, 0.1f, 0.75f);
    markers.push_back(m);
  }

  omniscope::RemoteLogger::instance().plot_markers(markers);
}

}  // namespace bridge
