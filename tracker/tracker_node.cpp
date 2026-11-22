#include "tracker_node.hpp"

#include <cmath>
#include <vector>

#include <Eigen/Geometry>
#include <omniscope/marker.hpp>

#include "fft.hpp"
#include "frames.hpp"

namespace tracker
{
namespace
{
constexpr double kLightbar = 56e-3;
constexpr double kArmorWidth = 135e-3;
constexpr double kArmorThick = 0.015;

omniscope::Vector3 to_vec3(const Eigen::Vector3d & v)
{
  return {static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())};
}

omniscope::Quaternion eigen_quat_to_omni(const Eigen::Quaterniond & q)
{
  omniscope::Quaternion oq;
  oq.x = static_cast<float>(q.x());
  oq.y = static_cast<float>(q.y());
  oq.z = static_cast<float>(q.z());
  oq.w = static_cast<float>(q.w());
  return oq;
}

omniscope::ColorRGBA to_rgba(float r, float g, float b, float a = 1.0f)
{
  return {r, g, b, a};
}

Eigen::Quaterniond armor_quat(double yaw)
{
  const double pitch = 15.0 * M_PI / 180.0;
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
}  // namespace

TrackerNode::TrackerNode(
  std::unique_ptr<EgoControlTracker> tracker,
  common::Channel<common::StampedObservation> & obs_in,
  common::LatestSlot<common::TrackerOutput> & est_out,
  common::LatestSlot<common::GroundTruth> & gt_in,
  double bullet_speed,
  common::AimerParams aimer,
  common::PlannerParams planner)
: tracker_(std::move(tracker)),
  obs_in_(obs_in),
  est_out_(est_out),
  gt_in_(gt_in),
  bullet_speed_(bullet_speed),
  planner_(aimer, planner)
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

void TrackerNode::publish_kalman_markers(
  const common::TargetEstimate & est, const Eigen::Vector3d & aim_lead_pt)
{
  std::vector<omniscope::Marker> markers;
  markers.reserve(16);
  const std::string frame = "gimbal";
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

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
      m.color = to_rgba(1.0f, 0.90f, 0.12f, 0.95f);
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
      m.color = to_rgba(1.0f, 0.90f, 0.12f, 1.0f);
      markers.push_back(m);
    }

    std::vector<omniscope::Vector3> ekf_spokes;
    for (size_t i = 0; i < est.armor_xyza_list.size(); i++) {
      Eigen::Vector3d p_plate(
        est.armor_xyza_list[i][0], est.armor_xyza_list[i][1], est.armor_xyza_list[i][2]);
      double yaw_plate = est.armor_xyza_list[i][3];

      omniscope::Marker m;
      m.frame_id = frame;
      m.ns = "kalman.armor";
      m.id = static_cast<int32_t>(60 + i);
      m.type = omniscope::MarkerType::CUBE;
      m.action = omniscope::MarkerAction::ADD;
      m.pose.position = to_vec3(p_plate);
      m.pose.orientation = eigen_quat_to_omni(armor_quat(yaw_plate));
      m.scale = {
        static_cast<float>(kArmorThick),
        static_cast<float>(kArmorWidth),
        static_cast<float>(kLightbar)};
      m.color = to_rgba(0.15f, 0.50f, 1.0f, 0.92f);
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
      m.color = to_rgba(0.15f, 0.50f, 1.0f, 0.65f);
      markers.push_back(m);
    }
  }

  {
    omniscope::Marker m;
    m.frame_id = frame;
    m.ns = "aim.lead";
    m.id = 80;
    m.type = omniscope::MarkerType::SPHERE;
    m.action = omniscope::MarkerAction::ADD;
    m.pose.position = to_vec3(aim_lead_pt);
    m.scale = {0.09f, 0.09f, 0.09f};
    m.color = to_rgba(1.0f, 0.35f, 0.7f, 1.0f);
    markers.push_back(m);
  }
  {
    omniscope::Marker m;
    m.frame_id = frame;
    m.ns = "aim.lead";
    m.id = 82;
    m.type = omniscope::MarkerType::LINE_LIST;
    m.action = omniscope::MarkerAction::ADD;
    m.points = {to_vec3(zero), to_vec3(aim_lead_pt)};
    m.scale = {0.008f, 0.0f, 0.0f};
    m.color = to_rgba(1.0f, 0.35f, 0.7f, 0.75f);
    markers.push_back(m);
  }

  omniscope::plot_markers(markers);
}

void TrackerNode::run()
{
  common::StampedObservation msg;
  double last_t = -1.0;

  while (!stop_requested_.load() && obs_in_.wait_pop(msg)) {
    if (last_t < 0.0) {
      tracker_->update(msg.obs);
    } else {
      tracker_->predict(msg.sim_t - last_t);
      tracker_->control(msg.ego_acc);
      tracker_->update(msg.obs);
    }
    last_t = msg.sim_t;

    auto target = tracker_->estimate();

    common::GroundTruth gt;
    common::TargetState enemy_g;
    bool have_gt = gt_in_.get(gt);
    if (have_gt && target.x.size() >= 11) {
      common::GroundTruth gt_g = gt;
      gt_g.target = common::express_target_in_gimbal(gt);
      enemy_g = gt_g.target;
      metrics_.update(gt_g, target, bullet_speed_);
    }
    const common::MetricsResult & metrics = metrics_.latest();

    common::GimbalCommand cmd;
    if (target.x.size() >= 11) {
      cmd = planner_.plan(target, bullet_speed_);
    }

    common::TrackerOutput out;
    out.est = target;
    out.sim_t = msg.sim_t;
    out.metrics = metrics;
    out.cmd = cmd;
    est_out_.set(out);

    if (target.x.size() >= 11) {
      omniscope::plot({
        {"kal_center_x", target.center.x()},
        {"kal_center_y", target.center.y()},
        {"kal_center_z", target.center.z()},
        {"kal_vel_x", target.velocity.x()},
        {"kal_vel_y", target.velocity.y()},
        {"kal_vel_z", target.velocity.z()},
        {"kal_yaw_deg", target.yaw * 180.0 / M_PI},
        {"kal_w", target.w},
        {"kal_radius_mm", target.radius * 1000.0},
        {"kal_l_mm", target.l * 1000.0},
        {"kal_h_mm", target.h * 1000.0},
      });
      if (have_gt) {
        omniscope::plot({
          {"true_l_mm", enemy_g.l * 1000.0},
          {"true_h_mm", enemy_g.h * 1000.0},
        });
      }
      omniscope::plot({
        {"err_x", metrics.err_x},
        {"err_y", metrics.err_y},
        {"err_z", metrics.err_z},
      });
      omniscope::plot({
        {"err_vx", metrics.err_vx},
        {"err_vy", metrics.err_vy},
        {"err_vz", metrics.err_vz},
        {"err_yaw_deg", metrics.err_yaw * 180.0 / M_PI},
        {"err_w", metrics.err_w},
        {"err_r_mm", metrics.err_r * 1000.0},
        {"err_l_mm", metrics.err_l * 1000.0},
        {"err_h_mm", metrics.err_h * 1000.0},
        {"err_pos", metrics.pos_err_norm},
        {"err_vel", metrics.vel_err_norm},
      });

      if (cmd.valid) {
        omniscope::plot({
          {"cmd_target_yaw_deg", cmd.target_yaw * 180.0 / M_PI},
          {"cmd_target_pitch_deg", cmd.target_pitch * 180.0 / M_PI},
          {"cmd_yaw_deg", cmd.yaw * 180.0 / M_PI},
          {"cmd_pitch_deg", cmd.pitch * 180.0 / M_PI},
          {"cmd_yaw_vel", cmd.yaw_vel},
          {"cmd_yaw_acc", cmd.yaw_acc},
          {"cmd_pitch_vel", cmd.pitch_vel},
          {"cmd_pitch_acc", cmd.pitch_acc},
          {"cmd_fly_time", cmd.fly_time},
        });
      }
      if (have_gt) {
        double true_aim_yaw = 0.0;
        if (gt.visible_armor_id >= 0 &&
            gt.visible_armor_id < static_cast<int>(enemy_g.armors_pos.size())) {
          const Eigen::Vector3d & p = enemy_g.armors_pos[static_cast<size_t>(gt.visible_armor_id)];
          true_aim_yaw = std::atan2(p.y(), p.x());
        }
        omniscope::plot({
          {"true_camera_yaw_deg", gt.camera.yaw * 180.0 / M_PI},
          {"true_aim_yaw_deg", true_aim_yaw * 180.0 / M_PI},
        });
      }

      const Eigen::Vector3d aim =
        cmd.valid ? cmd.aim_xyz : common::compute_aim_lead_point(target, metrics.flight_time);
      publish_kalman_markers(target, aim);
    }
  }

  finished_ = true;
}

}  // namespace tracker
