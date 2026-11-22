#include "sim_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#include <Eigen/Geometry>
#include <omniscope/marker.hpp>
#include <omniscope/omniscope.hpp>

#include "frames.hpp"

namespace sim
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

omniscope::TransformStamped make_tf(
  const std::string & parent,
  const std::string & child,
  const Eigen::Vector3d & translation,
  double yaw)
{
  omniscope::TransformStamped tf;
  tf.parent = parent;
  tf.child = child;
  tf.translation = to_vec3(translation);
  const Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  tf.rotation = eigen_quat_to_omni(q);
  return tf;
}

void publish_tfs(const common::GroundTruth & gt, uint64_t stamp_ns)
{
  omniscope::plot_tfs({
    make_tf("odom", "gimbal", gt.gimbal.pos, gt.gimbal.yaw),
    make_tf("gimbal", "camera", Eigen::Vector3d::Zero(), gt.camera.yaw),
    make_tf("odom", "robot", gt.target.center, gt.target.yaw),
  }, stamp_ns);
}

void publish_gt_markers(const common::GroundTruth & gt, uint64_t stamp_ns)
{
  std::vector<omniscope::Marker> markers;
  markers.reserve(20);

  {
    omniscope::Marker m;
    m.ns = "ground_truth.target";
    m.action = omniscope::MarkerAction::DELETEALL;
    markers.push_back(m);
  }
  {
    omniscope::Marker m;
    m.ns = "ground_truth.armor";
    m.action = omniscope::MarkerAction::DELETEALL;
    markers.push_back(m);
  }

  {
    omniscope::Marker m;
    m.frame_id = "odom";
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
    m.frame_id = "odom";
    m.ns = "my_robot.chassis";
    m.id = 1;
    m.type = omniscope::MarkerType::ARROW;
    m.action = omniscope::MarkerAction::ADD;
    const Eigen::Vector3d p = gt.chassis.pos;
    const Eigen::Vector3d tip = p + Eigen::Vector3d(
      0.40 * std::cos(gt.chassis.yaw), 0.40 * std::sin(gt.chassis.yaw), 0.0);
    m.points = {to_vec3(p), to_vec3(tip)};
    m.scale = {0.035f, 0.07f, 0.07f};
    m.color = to_rgba(0.0f, 0.94f, 1.0f, 1.0f);
    markers.push_back(m);
  }

  {
    omniscope::Marker m;
    m.frame_id = "gimbal";
    m.ns = "my_robot.gimbal";
    m.id = 11;
    m.type = omniscope::MarkerType::SPHERE;
    m.action = omniscope::MarkerAction::ADD;
    m.pose.position = {0.0f, 0.0f, 0.0f};
    m.scale = {0.11f, 0.11f, 0.11f};
    m.color = to_rgba(1.0f, 0.45f, 0.08f, 1.0f);
    markers.push_back(m);
  }
  {
    omniscope::Marker m;
    m.frame_id = "gimbal";
    m.ns = "my_robot.gimbal";
    m.id = 12;
    m.type = omniscope::MarkerType::ARROW;
    m.action = omniscope::MarkerAction::ADD;
    m.points = {{0.0f, 0.0f, 0.0f}, {0.50f, 0.0f, 0.0f}};
    m.scale = {0.025f, 0.05f, 0.05f};
    m.color = to_rgba(1.0f, 0.45f, 0.08f, 1.0f);
    markers.push_back(m);
  }

  {
    omniscope::Marker m;
    m.frame_id = "camera";
    m.ns = "my_robot.camera";
    m.id = 21;
    m.type = omniscope::MarkerType::SPHERE;
    m.action = omniscope::MarkerAction::ADD;
    m.pose.position = {0.0f, 0.0f, 0.0f};
    m.scale = {0.08f, 0.08f, 0.08f};
    m.color = to_rgba(0.45f, 0.75f, 1.0f, 1.0f);
    markers.push_back(m);
  }
  {
    omniscope::Marker m;
    m.frame_id = "camera";
    m.ns = "my_robot.camera";
    m.id = 22;
    m.type = omniscope::MarkerType::ARROW;
    m.action = omniscope::MarkerAction::ADD;
    m.points = {{0.0f, 0.0f, 0.0f}, {0.55f, 0.0f, 0.0f}};
    m.scale = {0.022f, 0.045f, 0.045f};
    m.color = to_rgba(0.45f, 0.75f, 1.0f, 1.0f);
    markers.push_back(m);
  }

  {
    omniscope::Marker m;
    m.frame_id = "robot";
    m.ns = "robot.center";
    m.id = 0;
    m.type = omniscope::MarkerType::SPHERE;
    m.action = omniscope::MarkerAction::ADD;
    m.pose.position = {0.0f, 0.0f, 0.0f};
    m.scale = {0.10f, 0.10f, 0.10f};
    m.color = to_rgba(0.12f, 0.95f, 0.22f, 0.95f);
    markers.push_back(m);
  }

  const int n = gt.target.armor_num;
  for (int i = 0; i < n; i++) {
    const Eigen::Vector3d p =
      common::armor_in_robot(i, n, gt.target.radius, gt.target.l, gt.target.h);
    const double yaw_i = common::armor_yaw_in_robot(i, n);
    omniscope::Marker m;
    m.frame_id = "robot";
    m.ns = "robot.armor";
    m.id = i;
    m.type = omniscope::MarkerType::CUBE;
    m.action = omniscope::MarkerAction::ADD;
    m.pose.position = to_vec3(p);
    m.pose.orientation = eigen_quat_to_omni(armor_quat(yaw_i));
    m.scale = {
      static_cast<float>(kArmorThick),
      static_cast<float>(kArmorWidth),
      static_cast<float>(kLightbar)};
    m.color = to_rgba(0.12f, 0.95f, 0.22f, 0.92f);
    markers.push_back(m);
  }

  omniscope::plot_markers(markers, stamp_ns);
}
}  // namespace

SimNode::SimNode(const common::SimulationConfig & cfg, Topics topics)
: cfg_(cfg),
  topics_(topics),
  kinematics_(cfg),
  dist_pos_(0.0, cfg.noise_pos_std),
  dist_yaw_(0.0, cfg.noise_yaw_std),
  dist_acc_(0.0, cfg.noise_acc_std)
{
}

void SimNode::start()
{
  thread_ = std::thread(&SimNode::run, this);
}

void SimNode::join()
{
  if (thread_.joinable()) thread_.join();
}

void SimNode::run()
{
  const int fps = std::max(cfg_.fps, 1);
  const double dt = 1.0 / static_cast<double>(fps);
  const auto frame_interval = std::chrono::microseconds(1000000 / fps);

  publish(0.0, 0);

  auto next_tick = std::chrono::steady_clock::now();
  while (!stop_requested_.load()) {
    kinematics_.step(dt);
    frame_++;
    const double sim_t = frame_ * dt;

    publish(sim_t, frame_);

    if (cfg_.duration > 0.0 && sim_t >= cfg_.duration) break;

    next_tick += frame_interval;
    std::this_thread::sleep_until(next_tick);
  }

  topics_.obs_out.close();
  finished_ = true;
}

common::ArmorObservation SimNode::observe(const common::GroundTruth & gt)
{
  common::ArmorObservation obs;
  obs.name = "one";
  obs.color = "blue";
  obs.type = "small";
  obs.confidence = 0.98;
  obs.width = 0.135;
  obs.height = 0.056;
  obs.armor_id = gt.visible_armor_id;

  const auto & armor_pos = gt.target.armors_pos[gt.visible_armor_id];
  const double armor_yaw_odom = gt.target.armors_yaw[gt.visible_armor_id];

  obs.xyz_in_gimbal = armor_pos - gt.gimbal.pos;
  obs.xyz_in_world = armor_pos;
  obs.yaw_raw = common::yaw_odom_to_gimbal(armor_yaw_odom, gt.gimbal);

  if (cfg_.enable_noise) {
    const Eigen::Vector3d noise(dist_pos_(rng_), dist_pos_(rng_), 0.0);
    obs.xyz_in_gimbal += noise;
    obs.yaw_raw += dist_yaw_(rng_);
  }

  const double d = obs.xyz_in_gimbal.norm();
  const double yaw = std::atan2(obs.xyz_in_gimbal.y(), obs.xyz_in_gimbal.x());
  const double pitch = std::atan2(obs.xyz_in_gimbal.z(), obs.xyz_in_gimbal.head<2>().norm());
  obs.ypd_in_world = {yaw, pitch, d};
  return obs;
}

void SimNode::publish(double sim_t, uint64_t frame)
{
  const uint64_t stamp_ns = omniscope::now_ns();
  common::GroundTruth gt = kinematics_.ground_truth();
  gt.t = sim_t;
  topics_.gt_out.set(gt);

  const auto obs = observe(gt);
  common::StampedObservation msg{obs, sim_t, frame, stamp_ns};
  const Eigen::Vector3d a = kinematics_.origin_acc();
  const Eigen::Vector3d u_true(-a.x(), -a.y(), 0.0);
  Eigen::Vector3d u = u_true;
  if (cfg_.enable_noise) {
    u.x() += dist_acc_(acc_rng_);
    u.y() += dist_acc_(acc_rng_);
  }
  msg.ego_acc = u;
  topics_.obs_out.push(msg);

  const common::TargetState enemy = common::express_target_in_gimbal(gt);
  Eigen::Vector3d true_armor = Eigen::Vector3d::Zero();
  double true_armor_yaw = 0.0;
  const int visible_id = gt.visible_armor_id;
  if (visible_id >= 0 && visible_id < static_cast<int>(enemy.armors_pos.size())) {
    true_armor = enemy.armors_pos[visible_id];
    true_armor_yaw = enemy.armors_yaw[visible_id];
  }

  omniscope::plot({
    {"obs_armor_x", obs.xyz_in_gimbal.x()},
    {"obs_armor_y", obs.xyz_in_gimbal.y()},
    {"obs_armor_z", obs.xyz_in_gimbal.z()},
  }, stamp_ns);
  omniscope::plot({
    {"true_center_x", enemy.center.x()},
    {"true_center_y", enemy.center.y()},
    {"true_center_z", enemy.center.z()},
  }, stamp_ns);
  omniscope::plot({
    {"obs_armor_yaw_deg", obs.yaw_raw * 180.0 / M_PI},
    {"obs_w_ego", gt.chassis.w},
    {"true_armor_x", true_armor.x()},
    {"true_armor_y", true_armor.y()},
    {"true_armor_z", true_armor.z()},
    {"true_armor_yaw_deg", true_armor_yaw * 180.0 / M_PI},
    {"true_vel_x", enemy.vel.x()},
    {"true_vel_y", enemy.vel.y()},
    {"true_vel_z", enemy.vel.z()},
    {"true_yaw_deg", enemy.yaw * 180.0 / M_PI},
    {"true_w", enemy.w},
    {"true_w_ego", gt.chassis.w},
    {"ego_acc_x", msg.ego_acc.x()},
    {"ego_acc_y", msg.ego_acc.y()},
  }, stamp_ns);
  omniscope::plot({
    {"u_true_x", u_true.x()},
    {"u_true_y", u_true.y()},
    {"u_x", u.x()},
    {"u_y", u.y()},
  }, stamp_ns);

  publish_tfs(gt, stamp_ns);
  publish_gt_markers(gt, stamp_ns);
}

}  // namespace sim
