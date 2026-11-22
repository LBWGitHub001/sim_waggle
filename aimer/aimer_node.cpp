#include "aimer_node.hpp"

#include <cmath>
#include <vector>

#include <omniscope/marker.hpp>
#include <omniscope/omniscope.hpp>

#include "frames.hpp"

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
: aimer_(aimer),
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

void AimerNode::publish_aim_markers(const Eigen::Vector3d & aim_lead_pt)
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

  omniscope::plot_markers(markers);
}

void AimerNode::run()
{
  common::TrackerOutput msg;

  while (!stop_requested_.load() && est_in_.wait_pop(msg)) {
    const auto & est = msg.est;
    common::GimbalCommand cmd;
    if (est.x.size() >= 11) {
      cmd = aimer_.aim(est, bullet_speed_);
    }
    cmd_out_.set(cmd);

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
      publish_aim_markers(cmd.aim_xyz);
    }

    common::GroundTruth gt;
    if (gt_in_.get(gt)) {
      const common::TargetState enemy_g = common::express_target_in_gimbal(gt);
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
  }

  finished_ = true;
}

}  // namespace aimer
