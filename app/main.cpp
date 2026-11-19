#include <csignal>
#include <chrono>
#include <thread>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <fmt/format.h>

#include "common/types.hpp"
#include "common/channel.hpp"
#include "common/metrics.hpp"
#include "sim/sim_node.hpp"
#include "tracker/baseline_tracker.hpp"
#include "tracker/tracker_node.hpp"
#include "render/camera_detector_renderer.hpp"
#include "bridge/omniscope_bridge.hpp"

namespace
{
std::atomic<bool> g_quit{false};

void signal_handler(int)
{
  g_quit = true;
}

// 应用层全部配置（在 load_config 一处解析，main 中不再有配置样板）
struct AppConfig
{
  common::SimulationConfig sim;
  bridge::BridgeConfig bridge;
  auto_aim::EkfParams ekf;
  Eigen::VectorXd p0;
};

AppConfig load_config(const std::string & path)
{
  const YAML::Node yaml = YAML::LoadFile(path);
  AppConfig cfg;

  // EKF 初始协方差缺省值（原版 auto_aim 参数）
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
  if (yaml["remote_logger"]) {
    auto rln = yaml["remote_logger"];
    cfg.bridge.control_port = rln["control_port"].as<uint16_t>(15000);
    cfg.bridge.beacon_port = rln["beacon_port"].as<uint16_t>(15999);
    cfg.bridge.enable_remote = rln["enable_remote"].as<bool>(true);
    cfg.bridge.enable_local = rln["enable_local"].as<bool>(true);
    cfg.bridge.log_dir = rln["log_dir"].as<std::string>("./logs");
    cfg.bridge.sender_name = rln["sender_name"].as<std::string>("sim_waggle");
    cfg.bridge.heartbeat_interval_ms = rln["heartbeat_interval_ms"].as<uint32_t>(500);
    cfg.bridge.beacon_interval_ms = rln["beacon_interval_ms"].as<uint32_t>(1000);
    cfg.bridge.img_width = rln["img_width"].as<int>(800);
    cfg.bridge.img_quality = rln["img_quality"].as<int>(60);
  }
  return cfg;
}
}  // namespace

int main(int argc, char * argv[])
{
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // 配置：默认 configs/sim_waggle.yaml，可用 argv[1] 指定其他 YAML 路径
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

  // ------------------------------------------------------------------
  // 1. 话题通道（类似 ROS topics，两节点唯一的耦合点）
  // ------------------------------------------------------------------
  common::Channel<common::StampedObservation> obs_channel;   // SimNode -> TrackerNode
  common::LatestSlot<common::GroundTruth> gt_slot;           // SimNode -> 可视化
  common::LatestSlot<common::StampedObservation> obs_slot;   // SimNode -> 可视化
  common::LatestSlot<common::TrackerOutput> est_slot;        // TrackerNode -> 可视化

  // ------------------------------------------------------------------
  // 2. 两个独立节点线程 + 可视化组件
  // ------------------------------------------------------------------
  sim::SimNode sim_node(cfg.sim, {obs_channel, gt_slot, obs_slot});

  auto tracker = std::make_unique<tracker::BaselineTracker>(
    cfg.sim.target_radius, cfg.sim.armor_num, cfg.ekf, cfg.p0);
  const std::string tracker_name = tracker->name();
  tracker::TrackerNode tracker_node(std::move(tracker), obs_channel, est_slot);

  bridge::OmniScopeBridge bridge(cfg.bridge);
  render::CameraDetectorRenderer renderer(800, 600, 60.0);
  common::SimulationMetrics metrics_evaluator;

  sim_node.start();
  tracker_node.start();

  bridge.log(
    "INFO", "sim_waggle started: tracker={}, chassis_w={:.1f}rad/s, offset={:.1f}cm, fps={}",
    tracker_name, cfg.sim.chassis_w, cfg.sim.gimbal_offset.norm() * 100.0, cfg.sim.fps);

  std::cout << "========================================================\n"
            << "  sim_waggle multi-thread node simulation running\n"
            << "  SimNode (50Hz) --> [FIFO Channel] --> TrackerNode\n"
            << "  Active Tracker:     " << tracker_name << "\n"
            << "  Chassis spin speed: " << cfg.sim.chassis_w << " rad/s\n"
            << "  Gimbal offset:      " << cfg.sim.gimbal_offset.norm() * 100.0 << " cm\n"
            << "  Target distance:    " << cfg.sim.target_init_pos.norm() << " m\n"
            << "  OmniScope Portal:   http://127.0.0.1:8080 (3D & Curves)\n"
            << "========================================================\n";

  // ------------------------------------------------------------------
  // 3. main 可视化循环：只读最新快照，不触碰节点内部状态
  // ------------------------------------------------------------------
  const auto frame_interval = std::chrono::microseconds(1000000 / std::max(cfg.sim.fps, 1));
  auto last_log = std::chrono::steady_clock::now();
  auto loop_t0 = last_log;

  while (!g_quit.load()) {
    auto tick_start = std::chrono::steady_clock::now();

    // duration 场景：两节点都收尾后退出
    if (sim_node.finished() && tracker_node.finished()) break;

    common::GroundTruth gt;
    common::TrackerOutput tout;
    common::StampedObservation obs_msg;
    const bool ready = gt_slot.get(gt) && est_slot.get(tout) && obs_slot.get(obs_msg);

    if (ready) {
      const auto & est = tout.est;

      // 误差指标与弹道超前落点（全部由快照计算）
      auto metrics = metrics_evaluator.update(gt, est, cfg.sim.bullet_speed);
      auto aim_lead_pt = common::compute_aim_lead_point(est, metrics.flight_time);

      // 发布：遥测曲线 + 3D Markers + 相机检测图像
      bridge.publish_telemetry(gt, obs_msg.obs, est, metrics);
      bridge.publish_3d_markers(gt, est, aim_lead_pt);
      cv::Mat frame = renderer.render(gt, est, obs_msg.obs, metrics);
      bridge.publish_camera_image(frame, "camera_detector");

      // 周期性终端打印（含观测队列深度，>1 说明 tracker 消费跟不上）
      if (tick_start - last_log >= std::chrono::seconds(2)) {
        last_log = tick_start;
        bridge.log(
          "INFO",
          "t={:.1f}s | abs: pos_err={:.3f}m vel_err={:.3f}m/s | rel: pos_err={:.3f}m vel_err={:.3f}m/s"
          " | rms(rel) p={:.3f}m v={:.3f}m/s | aim_bias={:.1f}cm nis={:.2f} obs_queue={}",
          tout.sim_t, metrics.pos_err_norm, metrics.vel_err_norm,
          metrics.rel_pos_err_norm, metrics.rel_vel_err_norm,
          metrics.rmse_pos_rel, metrics.rmse_vel_rel,
          metrics.aim_lead_bias * 100.0, est.last_nis, obs_channel.size());
        std::cout << fmt::format(
          "[t={:5.1f}s] ABS pos/vel err: {:5.3f} m / {:5.3f} m/s | REL pos/vel err: {:5.3f} m / {:5.3f} m/s"
          " | RMS(rel): {:5.3f} / {:5.3f} | Lead Bias: {:5.1f} cm | EKF w: {:5.2f} | Queue: {}\n",
          tout.sim_t, metrics.pos_err_norm, metrics.vel_err_norm,
          metrics.rel_pos_err_norm, metrics.rel_vel_err_norm,
          metrics.rmse_pos_rel, metrics.rmse_vel_rel,
          metrics.aim_lead_bias * 100.0, est.w, obs_channel.size());
      }
    }

    std::this_thread::sleep_until(tick_start + frame_interval);
  }

  // ------------------------------------------------------------------
  // 4. 收尾：停 SimNode（通道关闭）→ TrackerNode 排空退出 → join
  // ------------------------------------------------------------------
  sim_node.stop();
  sim_node.join();
  tracker_node.join();

  const double wall_s =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_t0).count();

  // 退出前输出整段运行的误差汇总（与 OmniScope 曲线同一口径）
  const auto & m = metrics_evaluator.latest();
  if (m.samples > 0) {
    std::cout << fmt::format(
      "\n===== estimation error summary ({} samples) =====\n"
      "  ABS  pos: rms={:.3f} m  mean={:.3f} m  max={:.3f} m\n"
      "  ABS  vel: rms={:.3f} m/s mean={:.3f} m/s max={:.3f} m/s\n"
      "  REL  pos: rms={:.3f} m      (vs target-gimbal relativeness)\n"
      "  REL  vel: rms={:.3f} m/s\n"
      "  lead bias (vel_err * flight_time) = {:.2f} mm\n",
      m.samples,
      m.rmse_pos, m.mean_pos, m.max_pos,
      m.rmse_vel, m.mean_vel, m.max_vel,
      m.rmse_pos_rel, m.rmse_vel_rel, m.aim_lead_bias * 1000.0);
    bridge.log(
      "INFO", "error summary: ABS pos rms={:.3f}m, ABS vel rms={:.3f}m/s, REL pos rms={:.3f}m, REL vel rms={:.3f}m/s",
      m.rmse_pos, m.rmse_vel, m.rmse_pos_rel, m.rmse_vel_rel);
  }

  bridge.log(
    "INFO", "sim_waggle stopped after {:.2f}s ({} frames published)",
    wall_s, sim_node.frames_published());
  bridge.shutdown();

  std::cout << "\nSimulation finished cleanly.\n";
  return 0;
}
