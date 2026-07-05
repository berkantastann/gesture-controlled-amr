#pragma once

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <memory>

/**
 * BT SyncActionNode: lift_joint'i hareket ettirir, hedefe ulaşana kadar bekler.
 *
 * XML kullanımı:
 *   <LiftControl name="lift_up"
 *                effort="100.0"
 *                target_position="0.14"
 *                timeout="8.0"
 *                hold_effort="100.0"/>
 *
 * Ports:
 *   effort          (double) — Hareket kuvveti (N).
 *   target_position (double) — Beklenen hedef joint pozisyonu (m). Belirtilirse
 *                              pozisyon hedefe ulaşana kadar bekler.
 *   tolerance       (double) — Hedef pozisyon toleransı (m). Varsayılan: 0.005
 *   timeout         (double) — target_position için max bekleme (s). Varsayılan: 8.0
 *   hold_effort     (double) — Hareketten sonra tutma kuvveti (N). Varsayılan: 0.0
 */
class LiftControl : public BT::SyncActionNode
{
public:
  LiftControl(
    const std::string & name,
    const BT::NodeConfiguration & config,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<double> shared_effort);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<double> shared_effort_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  double current_lift_pos_{0.0};
};
