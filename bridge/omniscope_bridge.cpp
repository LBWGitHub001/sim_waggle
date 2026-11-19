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

  // ---------------------------------------------------------------
  // 单位约定：长度 mm，速度 mm/s，角度 deg —— 同一张曲线图里的量级可比，
  // 曲线键名自带单位，图例即图注。
  // ---------------------------------------------------------------

  // ---------------------------------------------------------------
  // 1. 核心误差曲线（统一折算为"脱靶量"量纲 mm）：
  //    单独成包先发，看板首次注册这 6 条并默认勾选。
  //    两套口径：
  //      *_err      = 绝对误差（估计 vs 世界系真值，含云台公转平移）
  //      rel_*      = 相对误差（估计 vs 云台系相对真值 = 观测所在坐标系）
  //    速度误差按弹道飞行时间折算成落点脱靶量：*_lead = v_err * T
  // ---------------------------------------------------------------
  {
    nlohmann::json core;
    core["pos_err_mm"] = metrics.pos_err_norm * 1000.0;
    core["rel_pos_err_mm"] = metrics.rel_pos_err_norm * 1000.0;
    core["lead_bias_mm"] = metrics.aim_lead_bias * 1000.0;
    core["rel_lead_bias_mm"] = metrics.rel_aim_lead_bias * 1000.0;
    core["err_pos_x_mm"] = metrics.err_x * 1000.0;
    core["err_pos_y_mm"] = metrics.err_y * 1000.0;
    omniscope::RemoteLogger::instance().plot(core);
  }

  // ---------------------------------------------------------------
  // 2. 明细曲线（看板侧栏可逐条勾选）
  // ---------------------------------------------------------------
  {
    nlohmann::json detail;
    // 速度误差（mm/s）与相对分轴位置误差（mm）
    detail["vel_err_mm_s"] = metrics.vel_err_norm * 1000.0;
    detail["rel_vel_err_mm_s"] = metrics.rel_vel_err_norm * 1000.0;
    detail["err_vel_x_mm_s"] = metrics.err_vx * 1000.0;
    detail["err_vel_y_mm_s"] = metrics.err_vy * 1000.0;
    detail["rel_err_x_mm"] = metrics.rel_err_x * 1000.0;
    detail["rel_err_y_mm"] = metrics.rel_err_y * 1000.0;
    // 姿态 / 角速度 / 装甲板拟合
    detail["err_yaw_deg"] = metrics.err_yaw * 180.0 / M_PI;
    detail["err_w_deg_s"] = metrics.err_w * 180.0 / M_PI;
    detail["armor_err_mm"] = metrics.armor_err_norm * 1000.0;
    // 原始观测量与估计量（与误差曲线同一时间轴，便于对照成因；沿用原信道 SI 单位）
    detail["obs_x"] = obs.xyz_in_world.x();
    detail["obs_y"] = obs.xyz_in_world.y();
    detail["target_x"] = est.center.x();
    detail["target_y"] = est.center.y();
    detail["target_vx"] = est.velocity.x();
    detail["target_vy"] = est.velocity.y();
    detail["true_vx"] = gt.target.vel.x();
    detail["true_vy"] = gt.target.vel.y();
    // 滤波一致性
    detail["nis"] = metrics.nis;
    // 累计统计（自启动以来）
    detail["rmse_pos_mm"] = metrics.rmse_pos * 1000.0;
    detail["rmse_rel_pos_mm"] = metrics.rmse_pos_rel * 1000.0;
    detail["rmse_vel_mm_s"] = metrics.rmse_vel * 1000.0;
    detail["rmse_rel_vel_mm_s"] = metrics.rmse_vel_rel * 1000.0;
    omniscope::RemoteLogger::instance().plot(detail);
  }

  // ---------------------------------------------------------------
  // 3. 云台系 x-y 平面误差轨迹（XY 曲线 err_locus）
  //    误差随底盘转速公转时，轨迹呈圆环 —— 偏置影响的直观形态。
  //    限频 5Hz 刷新整窗，避免 UDP 带宽膨胀。
  // ---------------------------------------------------------------
  constexpr size_t kLocusWindow = 300;    // 窗口样本数（100Hz 下约 3s）
  constexpr uint32_t kLocusStride = 20;    // 每 20 帧刷新一次 → 5Hz

  locus_x_.push_back(metrics.rel_err_x);
  locus_y_.push_back(metrics.rel_err_y);
  if (locus_x_.size() > kLocusWindow) locus_x_.pop_front();
  if (locus_y_.size() > kLocusWindow) locus_y_.pop_front();

  if (++pub_frames_ % kLocusStride == 0 && locus_x_.size() >= 2) {
    nlohmann::json locus;
    locus["err_locus"] = {
      {"x", std::vector<double>(locus_x_.begin(), locus_x_.end())},
      {"y", std::vector<double>(locus_y_.begin(), locus_y_.end())},
      {"meta", {{"xlabel", "err_x (m)"}, {"kind", "locus"}}}};
    omniscope::RemoteLogger::instance().plot(locus);
  }
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
