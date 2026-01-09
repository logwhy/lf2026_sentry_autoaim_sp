#include "gimbal_ros.hpp"
#include "tools/math_tools.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace io
{
GimbalROS::GimbalROS() : Node("gimbal_ros_node")
{
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sub_opt;
  sub_opt.callback_group = callback_group_;

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/serial/imu", 10,
    std::bind(&GimbalROS::imu_callback, this, std::placeholders::_1),
    sub_opt);

  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/serial/gimbal_joint_state", 10,
    std::bind(&GimbalROS::joint_callback, this, std::placeholders::_1),
    sub_opt);

  cmd_pub_ = this->create_publisher<pb_rm_interfaces::msg::GimbalCmd>("cmd_gimbal", 10);
  shoot_pub_ = this->create_publisher<example_interfaces::msg::UInt8>("cmd_shoot", 10);
}

GimbalROS::~GimbalROS()
{
  if (spin_thread_.joinable()) spin_thread_.join();
  stop_spin();
}

void GimbalROS::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  auto t = std::chrono::steady_clock::now();
  // 注意：ROS2 四元数是 xyzw，Eigen 构造是 wxyz [cite: 1]
  Eigen::Quaterniond q_eigen(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  queue_.push({q_eigen, t});
}

void GimbalROS::joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.size() < 2) return;

  std::lock_guard<std::mutex> lock(mutex_);
  // 按照协议：position[0] 为 pitch, position[1] 为 yaw (弧度制)
  current_state_.pitch = static_cast<float>(msg->position[0]);
  current_state_.yaw = static_cast<float>(msg->position[1]);
  
  if (msg->velocity.size() >= 2) {
    current_state_.pitch_vel = static_cast<float>(msg->velocity[0]);
    current_state_.yaw_vel = static_cast<float>(msg->velocity[1]);
  }
}

Eigen::Quaterniond GimbalROS::q(std::chrono::steady_clock::time_point t)
{
  // 复用原插值逻辑，保证视觉处理延迟补偿 [cite: 1, 3]
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    double k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;
    return q_c;
  }
}

GimbalState GimbalROS::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_state_;
}

void GimbalROS::start_spin()
{
  if (spinning_.exchange(true)) return; // 已经在 spin

  executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(),  // options
    2                           // ✅ 线程数（你也可以调大）
  );

  auto self = this->shared_from_this(); // ✅ 现在安全了（必须在 shared_ptr 管理之后）
  executor_->add_node(self);

  spin_thread_ = std::thread([this]() {
    executor_->spin();
  });
}

void GimbalROS::stop_spin()
{
  if (!spinning_.exchange(false)) return;

  if (executor_) executor_->cancel();

  if (spin_thread_.joinable()) spin_thread_.join();

  if (executor_) {
    executor_->remove_node(this->shared_from_this()); // 可选
    executor_.reset();
  }
}
void GimbalROS::send(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc,
                     float pitch, float pitch_vel, float pitch_acc)
{
  // 1. 发布云台控制消息 (弧度制)
  auto gimbal_msg = pb_rm_interfaces::msg::GimbalCmd();
  gimbal_msg.header.stamp = this->now();
  gimbal_msg.yaw_type = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
  gimbal_msg.pitch_type = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;

  if (control) {
    gimbal_msg.position.yaw = yaw;
    gimbal_msg.position.pitch = pitch;
    gimbal_msg.velocity.yaw = yaw_vel;
    gimbal_msg.velocity.pitch = pitch_vel;
  } else {
    std::lock_guard<std::mutex> lock(mutex_);
    gimbal_msg.position.yaw = current_state_.yaw;
    gimbal_msg.position.pitch = current_state_.pitch;
  }
  cmd_pub_->publish(gimbal_msg);

  // 2. 发布开火消息 (0 或 1)
  auto shoot_msg = example_interfaces::msg::UInt8();
  shoot_msg.data = fire ? 1 : 0;
  shoot_pub_->publish(shoot_msg);
}

} // namespace io