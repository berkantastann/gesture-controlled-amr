#pragma once

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

/**
 * BT SyncActionNode: N saniye bekler, sonra SUCCESS döner.
 *
 * XML kullanımı:
 *   <WaitSeconds name="wait_at_B" seconds="5.0"/>
 *
 * Ports:
 *   seconds (double) — Kaç saniye bekleneceği.
 */
class WaitSeconds : public BT::SyncActionNode
{
public:
  WaitSeconds(
    const std::string & name,
    const BT::NodeConfiguration & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};
