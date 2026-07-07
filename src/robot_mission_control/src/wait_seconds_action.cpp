#include "robot_mission_control/wait_seconds_action.hpp"
#include <chrono>

WaitSeconds::WaitSeconds(
  const std::string & name,
  const BT::NodeConfiguration & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node)
{}

BT::PortsList WaitSeconds::providedPorts()
{
  return {
    BT::InputPort<double>("seconds", "Kac saniye bekleneceği")
  };
}

BT::NodeStatus WaitSeconds::tick()
{
  auto seconds_res = getInput<double>("seconds");
  if (!seconds_res) {
    RCLCPP_ERROR(node_->get_logger(),
      "[WaitSeconds '%s'] 'seconds' portu okunamadi: %s",
      name().c_str(), seconds_res.error().c_str());
    return BT::NodeStatus::FAILURE;
  }
  double seconds = seconds_res.value();

  RCLCPP_INFO(node_->get_logger(),
    "[WaitSeconds '%s'] %.1f saniye bekleniyor...", name().c_str(), seconds);

  auto end_time = std::chrono::steady_clock::now() +
                  std::chrono::duration<double>(seconds);
  rclcpp::Rate rate(10);
  while (std::chrono::steady_clock::now() < end_time && rclcpp::ok()) {
    rclcpp::spin_some(node_);  // lift effort timer'ı bekleme sirasinda da yayinlasin
    rate.sleep();
  }

  RCLCPP_INFO(node_->get_logger(),
    "[WaitSeconds '%s'] Bekleme tamamlandi", name().c_str());
  return BT::NodeStatus::SUCCESS;
}
