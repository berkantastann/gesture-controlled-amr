#include "robot_mission_control/navigate_to_pose_action.hpp"
#include <sstream>
#include <cmath>

NavigateToPose::NavigateToPose(
  const std::string & name,
  const BT::NodeConfiguration & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config), node_(node)
{
  client_ = rclcpp_action::create_client<NavAction>(node_, "navigate_to_pose");
}

BT::PortsList NavigateToPose::providedPorts()
{
  return {
    BT::InputPort<std::string>("goal", "Hedef konum: 'x;y;theta' formatinda")
  };
}

BT::NodeStatus NavigateToPose::onStart()
{
  goal_done_      = false;
  goal_succeeded_ = false;
  goal_handle_.reset();

  // --- 1. Port'tan goal string'i oku ---
  auto goal_res = getInput<std::string>("goal");
  if (!goal_res) {
    RCLCPP_ERROR(node_->get_logger(),
      "[NavigateToPose '%s'] 'goal' portu okunamadi: %s",
      name().c_str(), goal_res.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // --- 2. "x;y;theta" parse et ---
  std::stringstream ss(goal_res.value());
  std::string token;
  std::vector<double> parts;
  while (std::getline(ss, token, ';')) {
    try { parts.push_back(std::stod(token)); }
    catch (const std::exception &) {
      RCLCPP_ERROR(node_->get_logger(),
        "[NavigateToPose '%s'] parse hatasi: '%s'",
        name().c_str(), goal_res.value().c_str());
      return BT::NodeStatus::FAILURE;
    }
  }
  if (parts.size() != 3) {
    RCLCPP_ERROR(node_->get_logger(),
      "[NavigateToPose '%s'] 3 deger gerekli (x;y;theta), %zu bulundu",
      name().c_str(), parts.size());
    return BT::NodeStatus::FAILURE;
  }

  double x = parts[0], y = parts[1], theta = parts[2];

  // --- 3. PoseStamped oluştur ---
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id    = "map";
  pose.header.stamp       = node_->now();
  pose.pose.position.x    = x;
  pose.pose.position.y    = y;
  pose.pose.orientation.z = std::sin(theta / 2.0);
  pose.pose.orientation.w = std::cos(theta / 2.0);

  // --- 4. Nav2 action server hazır mı? ---
  // wait_for_action_server: server var mı? (goal göndermeyi değil, server'ın
  // varlığını kontrol eder). Nav2 30s delay ile çoktan ayaktaysa bu anında döner.
  if (!client_->wait_for_action_server(std::chrono::seconds(30))) {
    RCLCPP_ERROR(node_->get_logger(),
      "[NavigateToPose '%s'] Nav2 action server bulunamadi (30s beklendi)",
      name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // --- 5. Goal gönder — NON-BLOCKING ---
  // Önceki hatanın nedeni: async_send_goal future'ını blocking loop içinde
  // bekliyorduk. Bu loop spin_some()'u boğuyordu → callback gelemiyordu.
  //
  // Doğru yaklaşım: callback'leri tanımla, async_send_goal'ü çağır, hemen
  // RUNNING dön. Callback'ler tick döngüsündeki spin_some() ile işlenecek.

  NavAction::Goal goal_msg;
  goal_msg.pose = pose;

  auto options = rclcpp_action::Client<NavAction>::SendGoalOptions();

  // goal_response_callback: server'ın "kabul" veya "ret" yanıtı
  options.goal_response_callback =
    [this](GoalHandle::SharedPtr handle) {
      if (!handle) {
        RCLCPP_ERROR(node_->get_logger(),
          "[NavigateToPose '%s'] Nav2 goal'u reddetti!", name().c_str());
        goal_succeeded_ = false;
        goal_done_      = true;   // onRunning FAILURE dönsün
      } else {
        goal_handle_ = handle;    // onHalted için sakla
        RCLCPP_INFO(node_->get_logger(),
          "[NavigateToPose '%s'] Goal kabul edildi, robot gidiyor...", name().c_str());
      }
    };

  // result_callback: navigasyon bitti (başarılı veya değil)
  options.result_callback =
    [this](const GoalHandle::WrappedResult & result) {
      goal_succeeded_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
      goal_done_      = true;   // en son set et
      RCLCPP_INFO(node_->get_logger(),
        "[NavigateToPose '%s'] Navigasyon tamamlandi: %s",
        name().c_str(), goal_succeeded_ ? "BASARILI" : "BASARISIZ");
    };

  client_->async_send_goal(goal_msg, options);

  RCLCPP_INFO(node_->get_logger(),
    "[NavigateToPose '%s'] Goal gonderildi → x=%.2f y=%.2f theta=%.2f rad",
    name().c_str(), x, y, theta);

  return BT::NodeStatus::RUNNING;  // Hemen dön, callback'leri spin_some beklesin
}

BT::NodeStatus NavigateToPose::onRunning()
{
  // Her BT tick'inde çağrılır (100ms).
  // goal_done_ → result_callback veya goal_response_callback (reject) tarafından set edilir.
  // spin_some() bu callback'leri tick döngüsünde işler.
  if (!goal_done_) {
    return BT::NodeStatus::RUNNING;
  }
  return goal_succeeded_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void NavigateToPose::onHalted()
{
  // BT durdurulursa (Sequence başarısız vb.) robotu durdur.
  if (goal_handle_) {
    RCLCPP_INFO(node_->get_logger(),
      "[NavigateToPose '%s'] BT durduruldu, goal iptal ediliyor...", name().c_str());
    client_->async_cancel_goal(goal_handle_);
  }
}
