#include "aimer_node.hpp"

#include <cmath>
#include <vector>

#include <omniscope/marker.hpp>
#include <omniscope/omniscope.hpp>

namespace aimer
{
namespace
{
omniscope::Vector3 to_vec3(const Eigen::Vector3d & v)
{
  return {static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())};
}

omniscope::ColorRGBA to_rgba(float r, float g, float b, float a = 1.0f)
{
  return {r, g, b, a};
}
}  // namespace

AimerNode::AimerNode(
  common::AimerParams aimer,
  common::Channel<common::TrackerOutput> & est_in,
  common::LatestSlot<common::GroundTruth> & gt_in,
  common::LatestSlot<common::GimbalCommand> & cmd_out,
  double bullet_speed)
: aimer_est_(aimer),
  aimer_gt_(aimer),
  est_in_(est_in),
  gt_in_(gt_in),
  cmd_out_(cmd_out),
  bullet_speed_(bullet_speed)
{
}

void AimerNode::start()
{
  thread_ = std::thread(&AimerNode::run, this);
}

void AimerNode::join()
{
  if (thread_.joinable()) thread_.join();
}

common::GimbalCommand AimerNode::cmd_from_state(
  common::Aimer & aimer, const common::TargetEstimate & est)
{
  if (est.x.size() < 11) return {};
  return aimer.aim(est, bullet_speed_);
}

void AimerNode::publish_aim_markers(const Eigen::Vector3d & aim_lead_pt, uint64_t stamp_ns)
{
  std::vector<omniscope::Marker> markers;
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

  {
    omniscope::Marker m;
    m.frame_id = "gimbal";
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
    m.frame_id = "gimbal";
    m.ns = "aim.lead";
    m.id = 82;
    m.type = omniscope::MarkerType::LINE_LIST;
    m.action = omniscope::MarkerAction::ADD;
    m.points = {to_vec3(zero), to_vec3(aim_lead_pt)};
    m.scale = {0.008f, 0.0f, 0.0f};
    m.color = to_rgba(1.0f, 0.35f, 0.7f, 0.75f);
    markers.push_back(m);
  }

  omniscope::plot_markers(markers, stamp_ns);
}

void AimerNode::run()
{
  common::TrackerOutput msg;

  while (!stop_requested_.load() && est_in_.wait_pop(msg)) {
    const common::GimbalCommand cmd_est = cmd_from_state(aimer_est_, msg.est);
    cmd_out_.set(cmd_est);

    if (cmd_est.valid) {
      omniscope::plot({
        {"cmd_target_yaw_deg", cmd_est.target_yaw * 180.0 / M_PI},
        {"cmd_target_pitch_deg", cmd_est.target_pitch * 180.0 / M_PI},
        {"cmd_yaw_deg", cmd_est.yaw * 180.0 / M_PI},
        {"cmd_pitch_deg", cmd_est.pitch * 180.0 / M_PI},
        {"cmd_yaw_vel", cmd_est.yaw_vel},
        {"cmd_yaw_acc", cmd_est.yaw_acc},
        {"cmd_pitch_vel", cmd_est.pitch_vel},
        {"cmd_pitch_acc", cmd_est.pitch_acc},
        {"cmd_fly_time", cmd_est.fly_time},
      }, msg.stamp_ns);
      publish_aim_markers(cmd_est.aim_xyz, msg.stamp_ns);
    }

    common::GroundTruth gt;
    if (gt_in_.get(gt)) {
      const common::TargetEstimate gt_est = common::estimate_from_ground_truth(gt);
      const common::GimbalCommand cmd_true = cmd_from_state(aimer_gt_, gt_est);
      omniscope::plot({
        {"true_camera_yaw_deg", gt.camera.yaw * 180.0 / M_PI},
      }, msg.stamp_ns);
      if (cmd_true.valid) {
        omniscope::plot({
          {"true_aim_yaw_deg", cmd_true.yaw * 180.0 / M_PI},
          {"true_aim_pitch_deg", cmd_true.pitch * 180.0 / M_PI},
        }, msg.stamp_ns);
      }
    }
  }

  finished_ = true;
}

}  // namespace aimer
