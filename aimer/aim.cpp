#include "aim.hpp"

#include "frames.hpp"
#include "trajectory.hpp"

#include <cmath>
#include <vector>

namespace common
{
namespace
{
constexpr double kDeg = M_PI / 180.0;

void fill_derived(TargetEstimate & est)
{
  if (est.x.size() < 11) return;
  const Eigen::VectorXd & x = est.x;
  est.center = Eigen::Vector3d(x[0], x[2], x[4]);
  est.velocity = Eigen::Vector3d(x[1], x[3], x[5]);
  est.yaw = x[6];
  est.w = x[7];
  est.radius = x[8];
  est.l = x[9];
  est.h = x[10];

  const int n = (est.armor_num > 0) ? est.armor_num : 4;
  est.armor_num = n;
  est.armor_xyza_list.clear();
  est.armor_xyza_list.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; i++) {
    const Eigen::Vector3d xyz = h_armor_xyz(x, i, n);
    const double angle = wrap_pi(x[6] + static_cast<double>(i) * 2.0 * M_PI / n);
    est.armor_xyza_list.push_back({xyz.x(), xyz.y(), xyz.z(), angle});
  }
}
}  // namespace

Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id, int armor_num)
{
  const double angle = wrap_pi(x[6] + static_cast<double>(id) * 2.0 * M_PI / armor_num);
  const bool use_l_h = (armor_num == 4) && (id == 1 || id == 3);
  const double r = use_l_h ? x[8] + x[9] : x[8];
  const double z = use_l_h ? x[4] + x[10] : x[4];
  return {x[0] - r * std::cos(angle), x[2] - r * std::sin(angle), z};
}

TargetEstimate propagate_estimate(const TargetEstimate & est_in, double dt)
{
  TargetEstimate est = est_in;
  if (est.x.size() < 11) return est;

  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(11, 11);
  F(0, 1) = dt;
  F(2, 3) = dt;
  F(4, 5) = dt;
  F(6, 7) = dt;
  est.x = F * est.x;
  est.x[6] = wrap_pi(est.x[6]);
  fill_derived(est);
  return est;
}

namespace
{
TargetEstimate estimate_from_gimbal_target(const TargetState & enemy)
{
  TargetEstimate est;
  est.x = Eigen::VectorXd::Zero(11);
  est.x[0] = enemy.center.x();
  est.x[1] = enemy.vel.x();
  est.x[2] = enemy.center.y();
  est.x[3] = enemy.vel.y();
  est.x[4] = enemy.center.z();
  est.x[5] = enemy.vel.z();
  est.x[6] = enemy.yaw;
  est.x[7] = enemy.w;
  est.x[8] = enemy.radius;
  est.x[9] = enemy.l;
  est.x[10] = enemy.h;
  est.armor_num = enemy.armor_num > 0 ? enemy.armor_num : 4;
  est.jumped = true;
  fill_derived(est);
  return est;
}

GimbalCommand ballistic_aim_armor(
  TargetEstimate est, int armor_id, const AimerParams & params, double bullet_speed)
{
  GimbalCommand cmd;
  if (est.x.size() < 11) return cmd;
  if (bullet_speed < 14.0) bullet_speed = 23.0;
  if (params.delay_time > 0.0) est = propagate_estimate(est, params.delay_time);

  auto plate_xyz = [&](const TargetEstimate & e) -> Eigen::Vector3d {
    if (armor_id >= 0 && armor_id < static_cast<int>(e.armor_xyza_list.size())) {
      return e.armor_xyza_list[static_cast<size_t>(armor_id)].head<3>();
    }
    return e.center;
  };

  const Eigen::Vector3d xyz0 = plate_xyz(est);
  const double d0 = std::hypot(xyz0.x(), xyz0.y());
  BulletTrajectory traj0(bullet_speed, d0, xyz0.z());
  if (traj0.unsolvable) return cmd;

  double prev_fly_time = traj0.fly_time;
  BulletTrajectory current_traj = traj0;
  Eigen::Vector3d final_xyz = xyz0;
  for (int iter = 0; iter < 10; ++iter) {
    const TargetEstimate pred = propagate_estimate(est, prev_fly_time);
    final_xyz = plate_xyz(pred);
    const double d = std::hypot(final_xyz.x(), final_xyz.y());
    current_traj = BulletTrajectory(bullet_speed, d, final_xyz.z());
    if (current_traj.unsolvable) return cmd;
    if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) break;
    prev_fly_time = current_traj.fly_time;
  }

  cmd.valid = true;
  cmd.aim_xyz = final_xyz;
  cmd.fly_time = current_traj.fly_time;
  cmd.target_yaw = wrap_pi(std::atan2(final_xyz.y(), final_xyz.x()) + params.yaw_offset);
  cmd.target_pitch = -(current_traj.pitch + params.pitch_offset);
  cmd.yaw = cmd.target_yaw;
  cmd.pitch = cmd.target_pitch;
  return cmd;
}
}  // namespace

GimbalCommand aim_from_ground_truth(
  const GroundTruth & gt, const AimerParams & params, double bullet_speed)
{
  const TargetState enemy = express_target_in_gimbal(gt);
  const int id = gt.visible_armor_id;
  if (id < 0 || id >= enemy.armor_num) return {};
  return ballistic_aim_armor(estimate_from_gimbal_target(enemy), id, params, bullet_speed);
}

Aimer::Aimer(AimerParams params) : params_(params) {}

AimPoint Aimer::choose_aim_point(const TargetEstimate & est)
{
  const auto & plates = est.armor_xyza_list;
  if (plates.empty() || est.x.size() < 11) return {};

  if (!est.jumped) return {true, plates[0]};

  const int armor_num = static_cast<int>(plates.size());
  const double center_yaw = std::atan2(est.x[2], est.x[0]);

  std::vector<double> delta_angle_list;
  delta_angle_list.reserve(static_cast<size_t>(armor_num));
  for (int i = 0; i < armor_num; i++) {
    delta_angle_list.push_back(wrap_pi(plates[i][3] - center_yaw));
  }

  // |w|<=2 视为未小陀螺（auto_aim 源码误写成 x[8]=r，这里用角速度）
  if (std::abs(est.x[7]) <= 2.0) {
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[static_cast<size_t>(i)]) > 60.0 * kDeg) continue;
      id_list.push_back(i);
    }
    if (id_list.empty()) return {false, plates[0]};

    if (id_list.size() > 1) {
      const int id0 = id_list[0];
      const int id1 = id_list[1];
      if (lock_id_ != id0 && lock_id_ != id1) {
        lock_id_ = (std::abs(delta_angle_list[static_cast<size_t>(id0)]) <
                    std::abs(delta_angle_list[static_cast<size_t>(id1)]))
                     ? id0
                     : id1;
      }
      return {true, plates[static_cast<size_t>(lock_id_)]};
    }

    lock_id_ = -1;
    return {true, plates[static_cast<size_t>(id_list[0])]};
  }

  const double coming_angle = params_.coming_angle;
  const double leaving_angle = params_.leaving_angle;
  for (int i = 0; i < armor_num; i++) {
    const double da = delta_angle_list[static_cast<size_t>(i)];
    if (std::abs(da) > coming_angle) continue;
    if (est.x[7] > 0.0 && da < leaving_angle) return {true, plates[static_cast<size_t>(i)]};
    if (est.x[7] < 0.0 && da > -leaving_angle) return {true, plates[static_cast<size_t>(i)]};
  }

  return {false, plates[0]};
}

AimPoint Aimer::yaw_pitch(const TargetEstimate & est, double bullet_speed, Eigen::Vector2d & yp)
{
  AimPoint pt = choose_aim_point(est);
  if (!pt.valid) return pt;
  const Eigen::Vector3d xyz = pt.xyza.head<3>();
  const double d = std::hypot(xyz.x(), xyz.y());
  const BulletTrajectory traj(bullet_speed, d, xyz.z());
  if (traj.unsolvable) {
    pt.valid = false;
    return pt;
  }
  yp[0] = wrap_pi(std::atan2(xyz.y(), xyz.x()) + params_.yaw_offset);
  yp[1] = -(traj.pitch + params_.pitch_offset);
  return pt;
}

GimbalCommand Aimer::aim(TargetEstimate est, double bullet_speed)
{
  GimbalCommand cmd;
  if (est.x.size() < 11) return cmd;
  if (bullet_speed < 14.0) bullet_speed = 23.0;

  if (params_.delay_time > 0.0) est = propagate_estimate(est, params_.delay_time);

  AimPoint aim0 = choose_aim_point(est);
  if (!aim0.valid) return cmd;

  Eigen::Vector3d xyz0 = aim0.xyza.head<3>();
  double d0 = std::hypot(xyz0.x(), xyz0.y());
  BulletTrajectory traj0(bullet_speed, d0, xyz0.z());
  if (traj0.unsolvable) return cmd;

  double prev_fly_time = traj0.fly_time;
  BulletTrajectory current_traj = traj0;
  AimPoint aim_pt = aim0;

  for (int iter = 0; iter < 10; ++iter) {
    TargetEstimate pred = propagate_estimate(est, prev_fly_time);
    aim_pt = choose_aim_point(pred);
    if (!aim_pt.valid) return cmd;

    const Eigen::Vector3d xyz = aim_pt.xyza.head<3>();
    const double d = std::hypot(xyz.x(), xyz.y());
    current_traj = BulletTrajectory(bullet_speed, d, xyz.z());
    if (current_traj.unsolvable) return cmd;

    if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) break;
    prev_fly_time = current_traj.fly_time;
  }

  const Eigen::Vector3d final_xyz = aim_pt.xyza.head<3>();
  cmd.valid = true;
  cmd.aim_xyz = final_xyz;
  cmd.fly_time = current_traj.fly_time;
  cmd.target_yaw = wrap_pi(std::atan2(final_xyz.y(), final_xyz.x()) + params_.yaw_offset);
  cmd.target_pitch = -(current_traj.pitch + params_.pitch_offset);
  cmd.yaw = cmd.target_yaw;
  cmd.pitch = cmd.target_pitch;
  return cmd;
}

}  // namespace common
