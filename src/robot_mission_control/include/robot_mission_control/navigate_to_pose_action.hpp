#pragma once

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

/**
 * BT StatefulActionNode: Nav2 NavigateToPose action server'ına goal gönderir.
 *
 * XML kullanımı:
 *   <NavigateToPose name="go_to_A" goal="{point_A}"/>
 *
 * Ports:
 *   goal (string) — "x;y;theta" formatında hedef konum.
 *                   {point_A} şeklinde blackboard değişkeni olarak geçilir.
 *
 * StatefulActionNode seçildi çünkü:
 *   - Nav2'ye goal gönderdikten sonra robot dakikalarca sürebilir.
 *   - Bu süre boyunca BT "beklemeli" ama thread'i bloklamadan.
 *   - onStart() → goal gönder, RUNNING dön
 *   - onRunning() → her tick'te "tamamlandı mı?" kontrol et
 *   - onHalted() → BT iptal edilirse goal'ü cancel et
 */
class NavigateToPose : public BT::StatefulActionNode
{
public:
  using NavAction = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavAction>;

  NavigateToPose(
    const std::string & name,
    const BT::NodeConfiguration & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  // Goal gönder → RUNNING dön — Aşama 3'te implement edilecek
  BT::NodeStatus onStart() override;

  // Her tick: tamamlandı mı? — Aşama 3'te implement edilecek
  BT::NodeStatus onRunning() override;

  // BT durdurulursa goal'ü iptal et — Aşama 3'te implement edilecek
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavAction>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;

  bool goal_done_{false};
  bool goal_succeeded_{false};
};
