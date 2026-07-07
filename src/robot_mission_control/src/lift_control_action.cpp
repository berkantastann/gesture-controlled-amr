#include "robot_mission_control/lift_control_action.hpp"
#include <chrono>
#include <cmath>

LiftControl::LiftControl(
  const std::string & name,
  const BT::NodeConfiguration & config,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<double> shared_effort)
: BT::SyncActionNode(name, config), node_(node), shared_effort_(shared_effort)
{
  // /joint_states subscriber — lift_joint pozisyonunu takip eder.
  // Callback spin_some() ile işlenir, bu yüzden tick() içinde
  // spin_some() çağrılması gerekir.
  joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "lift_joint") {
          current_lift_pos_ = msg->position[i];
          break;
        }
      }
    });
}

BT::PortsList LiftControl::providedPorts()
{
  return {
    BT::InputPort<double>("effort",          "N cinsinden hareket kuvveti"),
    BT::InputPort<double>("target_position", "Hedef joint pozisyonu (m) — belirtilirse pozisyon beklenir"),
    BT::InputPort<double>("tolerance",       "Pozisyon toleransi (m), varsayilan: 0.005"),
    BT::InputPort<double>("timeout",         "Pozisyon bekleme max suresi (s), varsayilan: 8.0"),
    BT::InputPort<double>("hold_effort",     "Hareketten sonra tutma kuvveti (N), varsayilan: 0.0")
  };
}

BT::NodeStatus LiftControl::tick()
{
  // --- Port'ları oku ---
  auto effort_res = getInput<double>("effort");
  if (!effort_res) {
    RCLCPP_ERROR(node_->get_logger(),
      "[LiftControl '%s'] 'effort' portu okunamadi: %s",
      name().c_str(), effort_res.error().c_str());
    return BT::NodeStatus::FAILURE;
  }
  double effort      = effort_res.value();
  double hold_effort = getInput<double>("hold_effort").value_or(0.0);
  double tolerance   = getInput<double>("tolerance").value_or(0.005);
  double timeout     = getInput<double>("timeout").value_or(8.0);
  auto   target_opt  = getInput<double>("target_position");

  *shared_effort_ = effort;  // timer bu değeri yayınlamaya başlar

  if (target_opt.has_value()) {
    // --- Pozisyon tabanlı bekleme ---
    // spin_some() ile joint_state callback'leri işlenir,
    // current_lift_pos_ güncellenir.
    double target = target_opt.value();
    RCLCPP_INFO(node_->get_logger(),
      "[LiftControl '%s'] %.1fN — hedef pozisyon: %.3fm (mevcut: %.3fm)",
      name().c_str(), effort, target, current_lift_pos_);

    auto deadline = node_->now() + rclcpp::Duration::from_seconds(timeout);
    rclcpp::Rate rate(20);  // 20Hz: pozisyonu sık kontrol et

    while (rclcpp::ok()) {
      rclcpp::spin_some(node_);  // joint_state callback'ini işle

      if (std::abs(current_lift_pos_ - target) < tolerance) {
        RCLCPP_INFO(node_->get_logger(),
          "[LiftControl '%s'] Hedef pozisyona ulasildi: %.3fm", name().c_str(), current_lift_pos_);
        break;
      }
      if (node_->now() > deadline) {
        RCLCPP_WARN(node_->get_logger(),
          "[LiftControl '%s'] Timeout (%.1fs) — mevcut pozisyon: %.3fm",
          name().c_str(), timeout, current_lift_pos_);
        break;
      }
      rate.sleep();
    }

  } else {
    // --- Süre tabanlı bekleme (target_position verilmemişse) ---
    double duration = getInput<double>("timeout").value_or(2.0);
    RCLCPP_INFO(node_->get_logger(),
      "[LiftControl '%s'] %.1fN, %.1fs sure ile calistirilacak", name().c_str(), effort, duration);

    auto end_time = std::chrono::steady_clock::now() +
                    std::chrono::duration<double>(duration);
    rclcpp::Rate rate(10);
    while (std::chrono::steady_clock::now() < end_time && rclcpp::ok()) {
      rclcpp::spin_some(node_);  // lift effort timer'ı bekleme sirasinda da yayinlasin
      rate.sleep();
    }
  }

  // Tutma kuvvetine geç — timer bu değeri yayınlamaya devam eder
  *shared_effort_ = hold_effort;
  RCLCPP_INFO(node_->get_logger(),
    "[LiftControl '%s'] Tutma kuvveti aktif: %.1fN", name().c_str(), hold_effort);

  return BT::NodeStatus::SUCCESS;
}
