// Quantitative benchmark for sim_waggle: chassis-wobble induced pseudo velocity
// and candidate compensation schemes. Built standalone (no OmniScope/render).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "types.hpp"
#include "kinematics.hpp"
#include "camera_sensor.hpp"
#include "baseline_tracker.hpp"
#include "math_tools.hpp"

using Eigen::Vector3d;

namespace {

Eigen::Matrix3d rot_z(double yaw) {
  Eigen::Matrix3d R;
  const double c = std::cos(yaw), s = std::sin(yaw);
  R << c, -s, 0, s, c, 0, 0, 0, 1;
  return R;
}

struct Case {
  double w = 12.0;
  double offset = 0.05;
  double dist = 3.5;
  double target_w = 7.0;
  double q_xyz = 100.0;
  double q_yaw = 75.0;
  double r_yaw = 0.04;
  double r_pitch = 0.04;
  double r_dist = 0.015;
  bool noise = true;
  bool target_moves = false;
  double target_speed = 0.0;   // m/s, +y direction
  int fps = 100;
  double duration = 4.0;
};

enum Variant {
  BASE = 0,       // pseudo-world observation, stock aim
  FRAME = 1,      // observation translated into world frame (TF fix)
  LEAD_DISP = 2,  // pseudo-world filter + exact ego displacement over flight
  LEAD_NAIVE = 3, // pseudo-world filter + v_ego * T (naive add-back)
  LEAD_FULL = 4,  // pseudo-world filter + p_ego(t0) + exact ego displacement over T
};

const char * vname(Variant v) {
  switch (v) {
    case BASE: return "BASE";
    case FRAME: return "FRAME";
    case LEAD_DISP: return "LEAD_DISP";
    case LEAD_NAIVE: return "LEAD_NAIVE";
    case LEAD_FULL: return "LEAD_FULL";
  }
  return "?";
}

struct Result {
  double vabs = 0, vrel = 0, pabs = 0, prel = 0;
  double lead_lat = 0, lead_norm = 0;
  double ghost = 0, ghost_peak = 0;
  double nis = 0;
  int n = 0;
};

Result run_case(const Case & c, Variant variant) {
  common::SimulationConfig cfg;
  cfg.fps = c.fps;
  cfg.chassis_w = c.w;
  cfg.chassis_vx = 0.0;
  cfg.chassis_vy = 0.0;
  cfg.gimbal_offset = Vector3d(c.offset, 0.0, 0.0);
  cfg.target_init_pos = Vector3d(c.dist, 0.0, 0.0);
  cfg.target_vel = c.target_moves ? Vector3d(0.0, c.target_speed, 0.0) : Vector3d::Zero();
  cfg.target_radius = 0.20;
  cfg.target_w = c.target_w;
  cfg.armor_num = 4;
  cfg.enable_noise = c.noise;
  cfg.noise_pos_std = 0.005;
  cfg.noise_yaw_std = 0.01;
  cfg.duration = c.duration;

  auto_aim::EkfParams ekf;
  ekf.q_xyz_acc_var = c.q_xyz;
  ekf.q_yaw_acc_var = c.q_yaw;
  ekf.r_yaw = c.r_yaw;
  ekf.r_pitch = c.r_pitch;
  ekf.r_dist_base = c.r_dist;

  Eigen::VectorXd p0(11);
  p0 << 1, 64, 1, 64, 1, 64, 0.4, 400, 1.0e-5, 1, 1;

  sim::KinematicsEngine kin(cfg);
  sim::CameraSensor sensor(cfg);
  tracker::BaselineTracker tr(0.20, 4, ekf, p0);

  const double dt = 1.0 / c.fps;
  const int N = static_cast<int>(std::lround(c.duration * c.fps));
  const double bullet_speed = 25.0;

  auto obs0 = sensor.observe(kin.ground_truth());
  if (variant == FRAME) {
    // ground_truth() 已是世界系（gimbal.pos ≡ 0）；此分支保持历史对照，加平移现为 no-op
    obs0.xyz_in_world += kin.ground_truth().gimbal.pos;
    obs0.ypd_in_world = tools::xyz2ypd(obs0.xyz_in_world);
  }
  tr.update(obs0);

  Result r;
  for (int i = 1; i <= N; ++i) {
    kin.step(dt);
    const auto & gt = kin.ground_truth();
    auto obs = sensor.observe(gt);
    if (variant == FRAME) {
      obs.xyz_in_world += gt.gimbal.pos;
      obs.ypd_in_world = tools::xyz2ypd(obs.xyz_in_world);
    }
    tr.predict(dt);
    tr.update(obs);

    const double t = gt.t;
    if (t < 1.0) continue;  // skip convergence

    auto est = tr.estimate();
    const Vector3d v_true = gt.target.vel;
    const Vector3d v_g = gt.gimbal.vel;
    const Vector3d v_rel = v_true - v_g;

    const double vabs = (est.velocity - v_true).norm();
    const double vrel = (est.velocity - v_rel).norm();
    const double pabs = (est.center - gt.target.center).norm();
    const double prel = (est.center - (gt.target.center - gt.gimbal.pos)).norm();

    // Range from gimbal -> target, flight time.
    const double range = (gt.target.center - gt.gimbal.pos).norm();
    const double T = range / bullet_speed;

    // Centre lead prediction (isolates the translational ego-motion effect).
    Vector3d aim = est.center + est.velocity * T;
    if (variant == LEAD_DISP) {
      const Vector3d pg_future = gt.chassis.pos + gt.chassis.vel * T +
                                 rot_z(gt.chassis.yaw + gt.chassis.w * T) * cfg.gimbal_offset;
      aim += (pg_future - gt.gimbal.pos);
    } else if (variant == LEAD_NAIVE) {
      aim += v_g * T;
    } else if (variant == LEAD_FULL) {
      const Vector3d pg_future = gt.chassis.pos + gt.chassis.vel * T +
                                 rot_z(gt.chassis.yaw + gt.chassis.w * T) * cfg.gimbal_offset;
      aim += gt.gimbal.pos + (pg_future - gt.gimbal.pos);
    }

    const Vector3d true_future = gt.target.center + v_true * T;
    const Vector3d err = aim - true_future;
    const Vector3d los = (gt.target.center - gt.gimbal.pos).normalized();
    const Vector3d err_lat = err - err.dot(los) * los;

    r.vabs += vabs * vabs;
    r.vrel += vrel * vrel;
    r.pabs += pabs * pabs;
    r.prel += prel * prel;
    r.lead_lat += err_lat.squaredNorm();
    r.lead_norm += err.squaredNorm();
    r.ghost += v_g.norm();
    r.ghost_peak = std::max(r.ghost_peak, v_g.norm());
    r.nis += est.last_nis * est.last_nis;
    r.n++;
  }

  const double inv = r.n > 0 ? 1.0 / r.n : 0.0;
  r.vabs = std::sqrt(r.vabs * inv);
  r.vrel = std::sqrt(r.vrel * inv);
  r.pabs = std::sqrt(r.pabs * inv);
  r.prel = std::sqrt(r.prel * inv);
  r.lead_lat = std::sqrt(r.lead_lat * inv);
  r.lead_norm = std::sqrt(r.lead_norm * inv);
  r.ghost *= inv;
  r.nis = std::sqrt(r.nis * inv);
  return r;
}

void print_row(const Case & c, Variant v, const Result & r) {
  std::printf(
    "| %.2f | %.3f | %.2f | %.2f | %s | %.3f | %.3f | %.2f | %.2f | %.2f | %.2f | %.2f | %d |\n",
    c.w, c.offset, c.dist, c.r_yaw, vname(v), r.vabs, r.vrel, r.pabs, r.prel, r.ghost,
    r.lead_lat * 100.0, r.lead_norm * 100.0, r.n);
}

void header() {
  std::printf(
    "| w | off | dist | r_yaw | variant | v_abs_rms | v_rel_rms | p_abs_rms | p_rel_rms | ghost_mean | "
    "lead_lat_cm | lead_norm_cm | n |\n");
  std::printf(
    "|---|---|---|---|---|---|---|---|---|---|---|---|---|\n");
}

void run_all(const Case & c, const std::vector<Variant> & vs) {
  for (auto v : vs) print_row(c, v, run_case(c, v));
}

}  // namespace

int main(int argc, char ** argv) {
  std::string mode = argc > 1 ? argv[1] : "main";


if (mode == "trace") {
    Case c;
    c.offset = 0.10;
    c.r_yaw = 0.01;
    c.r_pitch = 0.01;
    c.noise = false;
    if (argc > 2) c.target_w = std::atof(argv[2]);
    // rebuild the same scenario, but print per-frame internals
    common::SimulationConfig cfg;
    cfg.fps = c.fps; cfg.chassis_w = c.w; cfg.gimbal_offset = Vector3d(c.offset, 0, 0);
    cfg.target_init_pos = Vector3d(c.dist, 0, 0); cfg.target_radius = 0.20;
    cfg.target_w = c.target_w; cfg.armor_num = 4; cfg.enable_noise = false;
    auto_aim::EkfParams ekf; ekf.q_xyz_acc_var = c.q_xyz; ekf.q_yaw_acc_var = c.q_yaw;
    ekf.r_yaw = c.r_yaw; ekf.r_pitch = c.r_pitch; ekf.r_dist_base = c.r_dist;
    Eigen::VectorXd p0(11); p0 << 1,64,1,64,1,64,0.4,400,1.0e-5,1,1;
    sim::KinematicsEngine kin(cfg); sim::CameraSensor sensor(cfg);
    tracker::BaselineTracker tr(0.20, 4, ekf, p0);
    const double dt = 1.0 / c.fps;
    tr.update(sensor.observe(kin.ground_truth()));
    std::printf("t,est_cx,est_cy,rel_cx,rel_cy,abs_cx,abs_cy,est_vx,est_vy,rel_vx,rel_vy,gx,gy,gvx,gvy,id\n");
    for (int i = 1; i <= 400; ++i) {
      kin.step(dt); const auto & gt = kin.ground_truth();
      auto obs = sensor.observe(gt);
      tr.predict(dt); tr.update(obs);
      auto est = tr.estimate();
      if (i % 5) continue;
      Vector3d relc = gt.target.center - gt.gimbal.pos;
      Vector3d relv = gt.target.vel - gt.gimbal.vel;
      std::printf("%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d\n",
        gt.t, est.center.x(), est.center.y(), relc.x(), relc.y(), gt.target.center.x(), gt.target.center.y(),
        est.velocity.x(), est.velocity.y(), relv.x(), relv.y(),
        gt.gimbal.pos.x(), gt.gimbal.pos.y(), gt.gimbal.vel.x(), gt.gimbal.vel.y(), gt.visible_armor_id);
    }
    return 0;
  }




  if (mode == "sens") {
    std::printf("### A. offset sweep, r_yaw=0.01 (w=12, d=3.5, static target)\n\n");
    header();
    for (double off : {0.0, 0.02, 0.05, 0.10, 0.20}) {
      Case c; c.offset = off; c.r_yaw = 0.01; c.r_pitch = 0.01;
      run_all(c, {BASE, FRAME});
    }
    std::printf("\n### B. spin sweep, r=0.10, r_yaw=0.01\n\n");
    header();
    for (double w : {0.0, 2.0, 4.0, 8.0, 12.0, 20.0}) {
      Case c; c.offset = 0.10; c.w = w; c.r_yaw = 0.01; c.r_pitch = 0.01;
      run_all(c, {BASE, FRAME});
    }
    std::printf("\n### C. spin sweep, r=0.10, r_yaw=0.04 (project default R)\n\n");
    header();
    for (double w : {0.0, 2.0, 4.0, 8.0, 12.0, 20.0}) {
      Case c; c.offset = 0.10; c.w = w; c.r_yaw = 0.04; c.r_pitch = 0.04;
      run_all(c, {BASE, FRAME});
    }
    return 0;
  }

  if (mode == "leadfull") {
    std::printf("### aim-stage compensation needs a fast relative-velocity estimate\n\n");
    header();
    for (double q : {10.0, 100.0, 1000.0}) {
      Case c;
      c.offset = 0.10; c.r_yaw = 0.01; c.r_pitch = 0.01; c.q_xyz = q;
      run_all(c, {BASE, LEAD_FULL, FRAME});
    }
    return 0;
  }

  if (mode == "qmove") {
    std::printf("### Q trade-off with a 1.5 m/s moving target (off=0.10, w=12, r_yaw=0.01)\n\n");
    header();
    for (double q : {1.0, 10.0, 100.0, 1000.0}) {
      for (bool moving : {false, true}) {
        Case c;
        c.offset = 0.10; c.r_yaw = 0.01; c.r_pitch = 0.01; c.q_xyz = q;
        c.target_moves = moving; c.target_speed = 1.5;
        run_all(c, {BASE, FRAME});
      }
    }
    std::printf("\n### default project config vs matched R (off=0.05, w=12)\n\n");
    header();
    for (double ry : {0.04, 0.01}) {
      Case c; c.offset = 0.05; c.r_yaw = ry; c.r_pitch = ry;
      run_all(c, {BASE, FRAME});
    }
    return 0;
  }

  if (mode == "main" || mode == "all") {
    std::printf("### 1. offset sweep (w=12 rad/s, d=3.5 m, r_yaw=0.04, noise on)\n\n");
    header();
    for (double off : {0.02, 0.05, 0.10, 0.20}) {
      Case c;
      c.offset = off;
      run_all(c, {BASE, FRAME, LEAD_DISP, LEAD_NAIVE});
    }
    std::printf("\n### 2. yaw-noise sweep (w=12, off=0.10, d=3.5)\n\n");
    header();
    for (double ry : {0.005, 0.01, 0.04}) {
      Case c;
      c.offset = 0.10;
      c.r_yaw = ry;
      c.r_pitch = ry;
      run_all(c, {BASE, FRAME, LEAD_DISP});
    }
    std::printf("\n### 3. chassis-w sweep (off=0.10, d=3.5, r_yaw=0.01)\n\n");
    header();
    for (double w : {0.0, 4.0, 8.0, 12.0, 20.0}) {
      Case c;
      c.offset = 0.10;
      c.w = w;
      c.r_yaw = 0.01;
      c.r_pitch = 0.01;
      run_all(c, {BASE, FRAME, LEAD_DISP});
    }
    std::printf("\n### 4. same, measurement noise OFF (deterministic signal)\n\n");
    header();
    for (double off : {0.02, 0.10, 0.20}) {
      Case c;
      c.offset = off;
      c.noise = false;
      run_all(c, {BASE, FRAME, LEAD_DISP});
    }
    std::printf("\n### 5. Q sweep (off=0.10, w=12, r_yaw=0.01)\n\n");
    header();
    for (double q : {1.0, 10.0, 100.0, 1000.0}) {
      Case c;
      c.offset = 0.10;
      c.r_yaw = 0.01;
      c.r_pitch = 0.01;
      c.q_xyz = q;
      run_all(c, {BASE});
    }
    std::printf("\n### 6. moving target + wobble (off=0.10, w=12, r_yaw=0.04, target 1.5 m/s +y)\n\n");
    header();
    for (bool moving : {false, true}) {
      Case c;
      c.offset = 0.10;
      c.target_moves = moving;
      c.target_speed = 1.5;
      run_all(c, {BASE, FRAME, LEAD_DISP});
    }
  }
  return 0;
}