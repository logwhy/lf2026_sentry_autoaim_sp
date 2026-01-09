#include <fmt/core.h>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <list>
#include <atomic>
#include <cmath>
#include <mutex>
#include <optional>
#include <yaml-cpp/yaml.h>
#include <future>

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
//#include "io/gimbal/gimbal.hpp"
#include "io/ros2/gimbal_ros.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"

#include "tasks/omniperception/decider.hpp"     // 决策模块
#include "tasks/omniperception/detection.hpp"   // DetectionResult

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

#include "tools/debug_tool.hpp"
#include "tools/viewer_tool.hpp"

using namespace std::chrono;

// 相机实例（双相机：cam1=主摄，cam2=后摄辅助）
//  - 读帧线程只负责更新 latest_frame/latest_ts
//  - 主循环按 perceptron 的逻辑：YOLO -> Decider::delta_angle -> DetectionResult 队列 -> Decider 过滤/排序/决策
struct CameraInstance {
  std::string name;               // "FRONT" / "BACK"
  std::unique_ptr<io::SNCamera> cam;
  std::unique_ptr<auto_aim::YOLO> yolo;
  double yaw_offset = 0.0;        // 例如后摄 +pi（rad）

  // latest frame (producer: read thread, consumer: main loop)
  std::mutex latest_mtx;
  cv::Mat latest_frame;
  steady_clock::time_point latest_ts{};
  bool has_latest = false;

  // 用于避免重复处理同一帧
  steady_clock::time_point last_processed_ts{};
};

// 候选结果：每路相机各自产生一个 DetectionResult（和 perceptron 一致），
// 同时保留 cam_idx 便于做“只允许主摄开火”等逻辑。
struct CamDetection {
  size_t cam_idx = 0;
  std::string cam_name;
  omniperception::DetectionResult dr;
};

//（参考 auto_aim_test_all）
static io::Command last_valid_command{};
static int consecutive_lost_frames = 0;
static constexpr int MAX_LOST_FRAMES = 8;
static bool target_stable = false;
static int stable_frame_count = 0;

// 添加角度连续性处理变量
static float last_sent_yaw = 0.0f;
static bool first_yaw_received = false;

// 最后一次“找到有效目标”的时间（用于额外hold）
static steady_clock::time_point last_found_time = steady_clock::now();

static float smooth_yaw_transition(float current_yaw, float last_yaw) {
  float diff = current_yaw - last_yaw;
  while (diff > M_PI) diff -= 2 * M_PI;
  while (diff < -M_PI) diff += 2 * M_PI;
  return last_yaw + diff;
}

int main(int argc, char* argv[]) {
  tools::Exiter exiter;
  std::string config_main = "configs/demo.yaml";

  rclcpp::init(argc, argv);

  using clock_t = std::chrono::steady_clock;

  // --- 读取 use_windows 配置 ---
  bool use_windows = false;
  try {
    use_windows = YAML::LoadFile(config_main)["use_windows"].as<bool>(false);
  } catch (...) {}

  // 云台通信
  //io::Gimbal gimbal(config_main);
  auto gimbal = std::make_shared<io::GimbalROS>();
  gimbal->start_spin();   // ✅ 构造完成后再启动自旋线程


  // 用于从 IMU 四元数计算当前云台 yaw（把相对角 delta_yaw 转成绝对 yaw 命令）
  auto_aim::Solver pose_solver(config_main);

  // 决策器：用来在两路相机候选之间选择“转动角度来自哪个相机”
  omniperception::Decider decider(config_main);

  // yaw_offset 维持你原来：后摄 +pi
  std::vector<std::unique_ptr<CameraInstance>> cams;
  cams.reserve(2);
  std::vector<std::pair<std::string, double>> cfg_list = {
    {"configs/cam1.yaml", 0.0},
    {"configs/cam2.yaml", M_PI}
  };

  for (size_t i = 0; i < cfg_list.size(); ++i) {
    auto ci = std::make_unique<CameraInstance>();
    ci->name = (i == 0 ? "FRONT" : "BACK");
    ci->cam = std::make_unique<io::SNCamera>(cfg_list[i].first);
    ci->yolo = std::make_unique<auto_aim::YOLO>(cfg_list[i].first);
    ci->yaw_offset = cfg_list[i].second;

    //tools::logger()->info("初始化相机: {}", ci->name);
    cams.emplace_back(std::move(ci));

    std::this_thread::sleep_for(seconds(2));
  }

  const size_t MAIN_CAM_IDX = 0; // cam1 = FRONT（主摄，可开火）
  const size_t AUX_CAM_IDX  = 1; // cam2 = BACK（后摄，仅辅助识别/转动）

  // 相机读帧线程：更新 latest_frame/latest_ts
  std::vector<std::thread> threads;
  threads.reserve(cams.size());
  for (auto& ci_ptr : cams) {
    threads.emplace_back([&ci_ptr, &exiter]() {
      cv::Mat f;
      steady_clock::time_point t;
      while (!exiter.exit()) {
        ci_ptr->cam->read(f, t);
        if (!f.empty()) {
          std::lock_guard<std::mutex> lk(ci_ptr->latest_mtx);
          ci_ptr->latest_frame = f.clone();
          ci_ptr->latest_ts = t;
          ci_ptr->has_latest = true;
        } else {
          std::this_thread::sleep_for(milliseconds(1));
        }
      }
    });
  }

  // Debug统计 + 可视化（解耦到 tools/）
  tools::DebugTool debug(cams.size());
  for (size_t i = 0; i < cams.size(); ++i) debug.set_cam_name(i, cams[i]->name);

  std::unique_ptr<tools::ViewerTool> viewer;
  std::vector<cv::Mat> current_frames;
  if (use_windows) {
    viewer = std::make_unique<tools::ViewerTool>("Dual Camera View", 0.5);
    current_frames.resize(cams.size());
  }

  while (!exiter.exit()) {
    debug.tick_main_loop();


    Eigen::Quaterniond q_now;
    {
      auto t0 = clock_t::now();
      q_now = gimbal->q(clock_t::now());
      
      auto t1 = clock_t::now();
      debug.add_gimbal_q_ms(duration<double, std::milli>(t1 - t0).count());
    }

    // 当前云台 yaw（rad）
    pose_solver.set_R_gimbal2world(q_now);
    const Eigen::Vector3d gimbal_pos = tools::eulers(pose_solver.R_gimbal2world(), 2, 1, 0);

    // ============
    // perceptron 同款：YOLO -> delta_angle -> DetectionResult
    // ============
    std::vector<CamDetection> dets;
    dets.reserve(cams.size());

    for (size_t i = 0; i < cams.size(); ++i) {
      static int back_tick = 0;
      if (i == AUX_CAM_IDX) {
        //continue 这里如果觉得帧数太小了可以直接把后摄给禁了，不过如果只用一个摄像头逻辑我已经写了，所以这里注释不用
        back_tick++;
        if (back_tick % 2 != 0) continue; // BACK 只跑 1/2 帧
      }

      cv::Mat img;
      steady_clock::time_point ts;
      bool has_new = false;

      {
        std::lock_guard<std::mutex> lk(cams[i]->latest_mtx);
        if (cams[i]->has_latest && cams[i]->latest_ts != cams[i]->last_processed_ts) {
          img = cams[i]->latest_frame;
          ts = cams[i]->latest_ts;
          cams[i]->last_processed_ts = cams[i]->latest_ts;
          has_new = true;
        }
      }

      if (!has_new || img.empty()) continue;

      auto now = clock_t::now();
      debug.record_cam_frame(i, duration<double, std::milli>(now - ts).count());

      if (use_windows) current_frames[i] = img;

      // YOLO detect
      std::list<auto_aim::Armor> armors;
      {
        auto t0 = clock_t::now();
        armors = cams[i]->yolo->detect(img, 0);
        auto t1 = clock_t::now();
        debug.add_yolo_detect_ms(duration<double, std::milli>(t1 - t0).count());
      }

      if (armors.empty()) continue;

      // camera tag：使用 "front"/"back"，对应你需要在 decider.delta_angle 里补分支
      const std::string cam_tag = (i == MAIN_CAM_IDX ? "front" : "back");
      const Eigen::Vector2d da_deg = decider.delta_angle(armors, cam_tag);

      omniperception::DetectionResult dr;
      dr.armors = armors;
      dr.timestamp = ts;
      dr.delta_yaw = tools::limit_rad(da_deg[0] / 57.3 );
      dr.delta_pitch = tools::limit_rad(da_deg[1] / 57.3);

      dets.push_back(CamDetection{i, cams[i]->name, dr});
    }

    
    // ============
    // 用 decider 的函数过滤/优先级 + 排序（保留 cam_idx 映射）
    // ============
    for (auto & cd : dets) {
      decider.armor_filter(cd.dr.armors);
      decider.set_priority(cd.dr.armors);
      if (!cd.dr.armors.empty()) {
        cd.dr.armors.sort([](const auto_aim::Armor & a, const auto_aim::Armor & b) {
          return a.priority < b.priority;
        });
      }
    }
    dets.erase(std::remove_if(dets.begin(), dets.end(), [](const CamDetection & cd) {
      return cd.dr.armors.empty();
    }), dets.end());

    std::sort(dets.begin(), dets.end(), [](const CamDetection & a, const CamDetection & b) {
      return a.dr.armors.front().priority < b.dr.armors.front().priority;
    });

    std::vector<omniperception::DetectionResult> detection_queue;
    detection_queue.reserve(dets.size());
    for (const auto & cd : dets) detection_queue.push_back(cd.dr);

    bool found = !detection_queue.empty();
    io::Command out_cmd = decider.decide(detection_queue); // yaw/pitch = 相对角(rad)
    size_t chosen_cam_idx = found ? dets.front().cam_idx : MAIN_CAM_IDX;

    // ============
    // 防丢帧 + 稳定开火（保留原风格）
    // ============
    if (found) {
      last_found_time = steady_clock::now();
      debug.add_found();

      const bool has_valid_target = out_cmd.control;
      if (has_valid_target) {
        consecutive_lost_frames = 0;

        // 不再区分相机来源：只要本帧有有效目标就累计稳定性
        stable_frame_count++;
        if (stable_frame_count >= 3) target_stable = true;

        if (std::isfinite(out_cmd.yaw) && std::isfinite(out_cmd.pitch)) {
          last_valid_command = out_cmd;
        }
      } else {
        consecutive_lost_frames++;
        stable_frame_count = 0;
        target_stable = false;
      }

      out_cmd.control = true;
      if (!std::isfinite(out_cmd.yaw) || !std::isfinite(out_cmd.pitch)) {
        out_cmd = last_valid_command;
        out_cmd.control = true;
        out_cmd.shoot = false;
        tools::logger()->warn("Invalid command, using last valid command");
      }

      // 开火只允许主摄 + 稳定 + yaw 波动小
      if (target_stable &&
          std::abs(out_cmd.yaw - last_valid_command.yaw) * 57.3 < 2.0) {
        out_cmd.shoot = true;
      } else {
        out_cmd.shoot = false;
      }

    } else {
      debug.add_lost();

      consecutive_lost_frames++;
      stable_frame_count = 0;
      target_stable = false;

      if (consecutive_lost_frames <= MAX_LOST_FRAMES) {
        out_cmd = last_valid_command;
        out_cmd.control = true;
        out_cmd.shoot = false;
      } else {
        out_cmd.control = false;
        out_cmd.shoot = false;
        out_cmd.yaw = 0;
        out_cmd.pitch = 0;
      }

      auto now = steady_clock::now();
      auto since_found_ms = duration_cast<milliseconds>(now - last_found_time).count();
      const int HOLD_MS = 200;
      if (since_found_ms > HOLD_MS) {
        out_cmd.control = false;
        out_cmd.shoot = false;
        out_cmd.yaw = 0;
        out_cmd.pitch = 0;
      }
    }
    // send（保持原通信方式）
    // 相对 yaw -> 绝对 yaw（对齐 decider.cpp 的旧做法：yaw 加 gimbal_pos[0]，pitch 不加）
    if (out_cmd.control) {
      // 1. 计算目标绝对角度 (当前云台角度 + 相对偏差)
      double target_yaw = out_cmd.yaw + gimbal_pos[0];

      // 2. 核心修改：强制归一化到 [-PI, PI] 也就是 [-180, 180]
      // 循环处理以应对多圈的情况
      while (target_yaw > M_PI) target_yaw -= 2.0 * M_PI;
      while (target_yaw < -M_PI) target_yaw += 2.0 * M_PI;

      out_cmd.yaw = static_cast<float>(target_yaw);

      auto t0 = clock_t::now();
      // 发送
      gimbal->send(out_cmd.control, out_cmd.shoot, out_cmd.yaw, 0, 0, out_cmd.pitch, 0, 0);
      auto t1 = clock_t::now();
      debug.add_gimbal_send_ms(duration<double, std::milli>(t1 - t0).count());
    }
    // 显示（可选，已解耦到 tools/viewer_tool.*）
    if (use_windows) {
      if (!viewer->update(current_frames)) break;
    }

    // ============
    // 1Hz 报告（已解耦到 tools/debug_tool.*）
    // ============
    debug.report_if_due(target_stable, consecutive_lost_frames, MAX_LOST_FRAMES);
  }

  for (auto& t : threads) t.join();

  rclcpp::shutdown();
  return 0;
}

