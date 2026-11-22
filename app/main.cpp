#include <atomic>
#include <csignal>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

#include <fmt/format.h>
#include <omniscope/omniscope.hpp>
#include <yaml-cpp/yaml.h>

#include "aimer/aimer_node.hpp"
#include "common/channel.hpp"
#include "common/types.hpp"
#include "sim/sim_node.hpp"
#include "tracker/tracker_node.hpp"

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
  tracker::EkfParams ekf;
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
    cfg.sim.bullet_speed = yaml["simulation"]["bullet_speed"].as<double>(25.0);
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
    cfg.sim.target_l = tr["l"].as<double>(0.05);
    cfg.sim.target_h = tr["h"].as<double>(0.05);
    cfg.sim.target_w = tr["w"].as<double>(4.0);
    cfg.sim.armor_num = tr["armor_num"].as<int>(4);
  }
  if (yaml["measurement"]) {
    auto ms = yaml["measurement"];
    cfg.sim.enable_noise = ms["enable_noise"].as<bool>(true);
    cfg.sim.noise_pos_std = ms["noise_pos_std"].as<double>(0.005);
    cfg.sim.noise_yaw_std = ms["noise_yaw_std"].as<double>(0.01);
  }
  if (yaml["ekf"]) {
    auto ekf_n = yaml["ekf"];
    cfg.ekf.q_xyz_acc_var = ekf_n["q_xyz_acc_var"].as<double>(100.0);
    cfg.ekf.q_yaw_acc_var = ekf_n["q_yaw_acc_var"].as<double>(75.0);
    cfg.ekf.r_yaw = ekf_n["r_yaw"].as<double>(0.04);
    cfg.ekf.r_pitch = ekf_n["r_pitch"].as<double>(0.04);
    cfg.ekf.r_dist_base = ekf_n["r_dist_base"].as<double>(0.015);
    if (ekf_n["p0"]) {
      auto p0_vec = ekf_n["p0"].as<std::vector<double>>();
      cfg.p0 = Eigen::Map<Eigen::VectorXd>(p0_vec.data(), p0_vec.size());
    }
  }
  if (yaml["aimer"]) {
    auto an = yaml["aimer"];
    constexpr double kDeg = M_PI / 180.0;
    cfg.sim.aimer.yaw_offset = an["yaw_offset"].as<double>(0.0) * kDeg;
    cfg.sim.aimer.pitch_offset = an["pitch_offset"].as<double>(0.0) * kDeg;
    cfg.sim.aimer.coming_angle = an["coming_angle"].as<double>(55.0) * kDeg;
    cfg.sim.aimer.leaving_angle = an["leaving_angle"].as<double>(20.0) * kDeg;
    cfg.sim.aimer.delay_time = an["delay_time"].as<double>(0.0);
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
  common::Channel<common::TrackerOutput> est_channel;
  common::LatestSlot<common::GimbalCommand> cmd_slot;

  sim::SimNode sim_node(cfg.sim, {obs_channel, gt_slot});
  tracker::TrackerNode tracker_node(
    cfg.sim.target_radius, cfg.sim.armor_num, cfg.ekf, cfg.p0, obs_channel, est_channel, gt_slot,
    cfg.sim.bullet_speed);
  aimer::AimerNode aimer_node(
    cfg.sim.aimer, est_channel, gt_slot, cmd_slot, cfg.sim.bullet_speed);

  auto & logger = omniscope::RemoteLogger::instance();
  logger.init(cfg.logger);

  sim_node.start();
  tracker_node.start();
  aimer_node.start();

  logger.log(
    "INFO", "sim_waggle started: chassis_w={:.1f}rad/s, offset={:.1f}cm, fps={}",
    cfg.sim.chassis_w, cfg.sim.gimbal_offset.norm() * 100.0, cfg.sim.fps);

  std::cout << "========================================================\n"
            << "  sim_waggle  SimNode --> TrackerNode --> AimerNode\n"
            << "  Chassis spin speed: " << cfg.sim.chassis_w << " rad/s\n"
            << "  Gimbal offset:      " << cfg.sim.gimbal_offset.norm() * 100.0 << " cm\n"
            << "  Target distance:    " << cfg.sim.target_init_pos.norm() << " m\n"
            << "  OmniScope Portal:   http://127.0.0.1:8080 (3D & Curves)\n"
            << "========================================================\n";

  auto last_log = std::chrono::steady_clock::now();
  auto loop_t0 = last_log;

  while (!g_quit.load()) {
    if (sim_node.finished() && tracker_node.finished() && aimer_node.finished()) break;

    auto now = std::chrono::steady_clock::now();
    if (now - last_log >= std::chrono::seconds(2)) {
      last_log = now;
      common::GroundTruth gt;
      common::GimbalCommand cmd;
      const bool have_gt = gt_slot.get(gt);
      const bool have_cmd = cmd_slot.get(cmd);
      if (have_gt) {
        logger.log(
          "INFO", "t={:.1f}s | true_w={:.2f} | cmd_yaw={:.2f}deg valid={} | obs_queue={}",
          gt.t, gt.target.w,
          have_cmd ? cmd.yaw * 180.0 / M_PI : 0.0, have_cmd && cmd.valid, obs_channel.size());
        std::cout << fmt::format(
          "[t={:5.1f}s] true_w={:5.2f} | cmd_yaw={:6.2f} deg valid={} | Queue: {}\n",
          gt.t, gt.target.w, have_cmd ? cmd.yaw * 180.0 / M_PI : 0.0, have_cmd && cmd.valid,
          obs_channel.size());
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  sim_node.stop();
  sim_node.join();
  tracker_node.join();
  aimer_node.join();

  const double wall_s =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_t0).count();

  logger.log("INFO", "sim_waggle stopped after {:.2f}s ({} frames published)", wall_s,
             sim_node.frames_published());
  logger.shutdown();

  std::cout << "\nSimulation finished cleanly.\n";
  return 0;
}
