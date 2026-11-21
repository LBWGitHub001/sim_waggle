#include <algorithm>
#include <atomic>
#include <csignal>
#include <chrono>
#include <cmath>
#include <thread>
#include <iostream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fmt/format.h>
#include <omniscope/omniscope.hpp>

#include "common/types.hpp"
#include "common/channel.hpp"
#include "sim/sim_node.hpp"
#include "tracker/ego_control_tracker.hpp"
#include "tracker/tracker_node.hpp"
#include "render/camera_detector_renderer.hpp"

namespace
{
std::atomic<bool> g_quit{false};

void signal_handler(int)
{
  g_quit = true;
}

struct AppConfig
{
  common::SimulationConfig sim;
  omniscope::RemoteLogger::Config logger;
  auto_aim::EkfParams ekf;
  Eigen::VectorXd p0;
};

AppConfig load_config(const std::string & path)
{
  const YAML::Node yaml = YAML::LoadFile(path);
  AppConfig cfg;

  cfg.p0.resize(11);
  cfg.p0 << 1, 64, 1, 64, 1, 64, 0.4, 400, 1.0e-5, 1, 1;

  if (yaml["simulation"]) {
    cfg.sim.fps = yaml["simulation"]["fps"].as<int>(50);
    cfg.sim.duration = yaml["simulation"]["duration"].as<double>(0.0);
  }
  if (yaml["my_robot"]) {
    auto mr = yaml["my_robot"];
    cfg.sim.chassis_w = mr["chassis_w"].as<double>(10.0);
    cfg.sim.chassis_vx = mr["chassis_vx"].as<double>(0.0);
    cfg.sim.chassis_vy = mr["chassis_vy"].as<double>(0.0);
    auto off = mr["gimbal_offset"].as<std::vector<double>>();
    cfg.sim.gimbal_offset = Eigen::Vector3d(off[0], off[1], off[2]);
  }
  if (yaml["target_robot"]) {
    auto tr = yaml["target_robot"];
    auto init_pos = tr["init_pos"].as<std::vector<double>>();
    cfg.sim.target_init_pos = Eigen::Vector3d(init_pos[0], init_pos[1], init_pos[2]);
    auto vel = tr["vel"].as<std::vector<double>>();
    cfg.sim.target_vel = Eigen::Vector3d(vel[0], vel[1], vel[2]);
    cfg.sim.target_radius = tr["radius"].as<double>(0.20);
    cfg.sim.target_w = tr["w"].as<double>(4.0);
    cfg.sim.armor_num = tr["armor_num"].as<int>(4);
  }
  if (yaml["measurement"]) {
    auto ms = yaml["measurement"];
    cfg.sim.enable_noise = ms["enable_noise"].as<bool>(true);
    cfg.sim.noise_pos_std = ms["noise_pos_std"].as<double>(0.005);
    cfg.sim.noise_yaw_std = ms["noise_yaw_std"].as<double>(0.01);
    cfg.sim.noise_acc_std = ms["noise_acc_std"].as<double>(1.0);
  }
  if (yaml["ekf"]) {
    auto ekf_n = yaml["ekf"];
    cfg.ekf.q_xyz_acc_var = ekf_n["q_xyz_acc_var"].as<double>(100.0);
    cfg.ekf.q_yaw_acc_var = ekf_n["q_yaw_acc_var"].as<double>(75.0);
    cfg.ekf.q_u_acc_var = ekf_n["q_u_acc_var"].as<double>(1.0);
    cfg.ekf.r_yaw = ekf_n["r_yaw"].as<double>(0.04);
    cfg.ekf.r_pitch = ekf_n["r_pitch"].as<double>(0.04);
    cfg.ekf.r_dist_base = ekf_n["r_dist_base"].as<double>(0.015);
    if (ekf_n["p0"]) {
      auto p0_vec = ekf_n["p0"].as<std::vector<double>>();
      cfg.p0 = Eigen::Map<Eigen::VectorXd>(p0_vec.data(), p0_vec.size());
    }
  }
  if (yaml["remote_logger"]) {
    auto rln = yaml["remote_logger"];
    cfg.logger.control_port = rln["control_port"].as<uint16_t>(15000);
    cfg.logger.beacon_port = rln["beacon_port"].as<uint16_t>(15999);
    cfg.logger.enable_remote = rln["enable_remote"].as<bool>(true);
    cfg.logger.enable_local = rln["enable_local"].as<bool>(true);
    cfg.logger.log_dir = rln["log_dir"].as<std::string>("./logs");
    cfg.logger.sender_name = rln["sender_name"].as<std::string>("sim_waggle");
    cfg.logger.heartbeat_interval_ms = rln["heartbeat_interval_ms"].as<uint32_t>(500);
    cfg.logger.beacon_interval_ms = rln["beacon_interval_ms"].as<uint32_t>(1000);
    cfg.logger.img_width = rln["img_width"].as<int>(800);
    cfg.logger.img_quality = rln["img_quality"].as<int>(60);
  }
  return cfg;
}
}  // namespace

int main(int argc, char * argv[])
{
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  if (argc > 2) {
    std::cerr << "Usage: " << argv[0] << " [config.yaml]" << std::endl;
    return 1;
  }
  const std::string config_path = (argc == 2) ? argv[1] : "configs/sim_waggle.yaml";

  AppConfig cfg;
  try {
    cfg = load_config(config_path);
  } catch (const std::exception & e) {
    std::cerr << "Failed to load config file: " << config_path << ": " << e.what() << std::endl;
    return 1;
  }

  common::Channel<common::StampedObservation> obs_channel;
  common::LatestSlot<common::GroundTruth> gt_slot;
  common::LatestSlot<common::StampedObservation> obs_slot;
  common::LatestSlot<common::TrackerOutput> est_slot;

  sim::SimNode sim_node(cfg.sim, {obs_channel, gt_slot, obs_slot});

  auto tracker = std::make_unique<tracker::EgoControlTracker>(
    cfg.sim.target_radius, cfg.sim.armor_num, cfg.ekf, cfg.p0);
  const std::string tracker_name = tracker->name();
  tracker::TrackerNode tracker_node(
    std::move(tracker), obs_channel, est_slot, gt_slot, cfg.sim.bullet_speed);

  auto & logger = omniscope::RemoteLogger::instance();
  logger.init(cfg.logger);

  render::CameraDetectorRenderer renderer(800, 600, 60.0);

  sim_node.start();
  tracker_node.start();

  logger.log(
    "INFO", "sim_waggle started: tracker={}, chassis_w={:.1f}rad/s, offset={:.1f}cm, fps={}",
    tracker_name, cfg.sim.chassis_w, cfg.sim.gimbal_offset.norm() * 100.0, cfg.sim.fps);

  std::cout << "========================================================\n"
            << "  sim_waggle multi-thread node simulation running\n"
            << "  SimNode --> [FIFO Channel] --> TrackerNode\n"
            << "  Active Tracker:     " << tracker_name << "\n"
            << "  Chassis spin speed: " << cfg.sim.chassis_w << " rad/s\n"
            << "  Gimbal offset:      " << cfg.sim.gimbal_offset.norm() * 100.0 << " cm\n"
            << "  Target distance:    " << cfg.sim.target_init_pos.norm() << " m\n"
            << "  OmniScope Portal:   http://127.0.0.1:8080 (3D & Curves)\n"
            << "========================================================\n";

  const auto frame_interval = std::chrono::microseconds(1000000 / std::max(cfg.sim.fps, 1));
  auto last_log = std::chrono::steady_clock::now();
  auto loop_t0 = last_log;
  common::MetricsResult last_metrics;

  while (!g_quit.load()) {
    auto tick_start = std::chrono::steady_clock::now();

    if (sim_node.finished() && tracker_node.finished()) break;

    common::GroundTruth gt;
    common::TrackerOutput tout;
    common::StampedObservation obs_msg;
    const bool ready = gt_slot.get(gt) && est_slot.get(tout) && obs_slot.get(obs_msg);

    if (ready) {
      const auto & est = tout.est;
      const auto & metrics = tout.metrics;
      last_metrics = metrics;

      cv::Mat frame = renderer.render(gt, est, obs_msg.obs, metrics);
      logger.plot_image(frame, {{"name", "camera_detector"}});

      if (tick_start - last_log >= std::chrono::seconds(2)) {
        last_log = tick_start;
        logger.log(
          "INFO",
          "t={:.1f}s | kal-true: pos={:.3f}m vel={:.3f}m/s yaw={:.2f}deg"
          " | rms p={:.3f}m v={:.3f}m/s | aim_bias={:.1f}cm nis={:.2f} obs_queue={}",
          tout.sim_t, metrics.pos_err_norm, metrics.vel_err_norm,
          metrics.err_yaw * 180.0 / M_PI,
          metrics.rmse_pos, metrics.rmse_vel,
          metrics.aim_lead_bias * 100.0, est.last_nis, obs_channel.size());
        std::cout << fmt::format(
          "[t={:5.1f}s] kal-true pos/vel: {:5.3f} m / {:5.3f} m/s | yaw {:5.2f} deg"
          " | RMS: {:5.3f} / {:5.3f} | Lead Bias: {:5.1f} cm | EKF w: {:5.2f} | Queue: {}\n",
          tout.sim_t, metrics.pos_err_norm, metrics.vel_err_norm,
          metrics.err_yaw * 180.0 / M_PI,
          metrics.rmse_pos, metrics.rmse_vel,
          metrics.aim_lead_bias * 100.0, est.w, obs_channel.size());
      }
    }

    std::this_thread::sleep_until(tick_start + frame_interval);
  }

  sim_node.stop();
  sim_node.join();
  tracker_node.join();

  const double wall_s =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_t0).count();

  const auto & m = last_metrics;
  if (m.samples > 0) {
    std::cout << fmt::format(
      "\n===== kalman − truth ({} samples) =====\n"
      "  pos: rms={:.3f} m  mean={:.3f} m  max={:.3f} m\n"
      "  vel: rms={:.3f} m/s mean={:.3f} m/s max={:.3f} m/s\n"
      "  aim |kal-true| = {:.2f} mm\n",
      m.samples,
      m.rmse_pos, m.mean_pos, m.max_pos,
      m.rmse_vel, m.mean_vel, m.max_vel,
      m.aim_lead_bias * 1000.0);
    logger.log(
      "INFO", "error summary: pos rms={:.3f}m, vel rms={:.3f}m/s, aim={:.2f}mm",
      m.rmse_pos, m.rmse_vel, m.aim_lead_bias * 1000.0);
  }

  logger.log(
    "INFO", "sim_waggle stopped after {:.2f}s ({} frames published)",
    wall_s, sim_node.frames_published());
  logger.shutdown();

  std::cout << "\nSimulation finished cleanly.\n";
  return 0;
}
