#include "tracker_node.hpp"

#include <cmath>
#include <vector>

#include <Eigen/Geometry>
#include <omniscope/marker.hpp>
#include <omniscope/omniscope.hpp>

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
  double target_radius,
  int armor_num,
  EkfParams ekf,
  Eigen::VectorXd p0,
  common::Channel<common::StampedObservation> & obs_in,
  common::Channel<common::TrackerOutput> & est_out,
  common::LatestSlot<common::GroundTruth> & gt_in,
  double bullet_speed)
: target_(target_radius, armor_num, std::move(p0), ekf),
  obs_in_(obs_in),
  est_out_(est_out),
  gt_in_(gt_in),
  bullet_speed_(bullet_speed)
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

void TrackerNode::publish_kalman_markers(const common::TargetEstimate & est, uint64_t stamp_ns)
{
  std::vector<omniscope::Marker> markers;
  markers.reserve(16);
  const std::string frame = "gimbal";

  if (est.x.size() < 11) return;

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

  omniscope::plot_markers(markers, stamp_ns);
}

void TrackerNode::run()
{
  common::StampedObservation msg;
  double last_t = -1.0;

  while (!stop_requested_.load() && obs_in_.wait_pop(msg)) {
    if (!target_.initialized()) {
      target_.init(msg.obs);
    } else {
      target_.predict(msg.sim_t - last_t);
      target_.control(msg.ego_acc);
      target_.update(msg.obs);
    }
    last_t = msg.sim_t;

    const auto target = target_.estimate();

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

    common::TrackerOutput out;
    out.est = target;
    out.sim_t = msg.sim_t;
    out.stamp_ns = msg.stamp_ns;
    out.metrics = metrics;
    est_out_.push(out);

    if (target.x.size() >= 11) {
      omniscope::plot({
        {"kal_center_x", target.center.x()},
        {"kal_center_y", target.center.y()},
        {"kal_vel_x", target.velocity.x()},
        {"kal_vel_y", target.velocity.y()},
        {"kal_yaw_deg", target.yaw * 180.0 / M_PI},
        {"kal_w", target.w},
      }, msg.stamp_ns);
      publish_kalman_markers(target, msg.stamp_ns);
    }
  }

  est_out_.close();
  finished_ = true;
}

}  // namespace tracker
