#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "robot_mission_control/lift_control_action.hpp"
#include "robot_mission_control/navigate_to_pose_action.hpp"
#include "robot_mission_control/wait_seconds_action.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mission_executor");

  // ─────────────────────────────────────────────────
  // 1. ROS PARAMETRELERİ
  // ─────────────────────────────────────────────────
  node->declare_parameter("point_A",    "3.0;0.0;0.0");
  node->declare_parameter("point_B",    "0.0;3.0;1.5708");
  node->declare_parameter("bt_xml_path", "");

  const std::string point_A     = node->get_parameter("point_A").as_string();
  const std::string point_B     = node->get_parameter("point_B").as_string();
  const std::string bt_xml_path = node->get_parameter("bt_xml_path").as_string();

  if (bt_xml_path.empty()) {
    RCLCPP_FATAL(node->get_logger(),
      "bt_xml_path parametresi bos! mission.launch.py'yi kontrol et.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Gorev noktalari yuklendi:");
  RCLCPP_INFO(node->get_logger(), "  point_A = %s", point_A.c_str());
  RCLCPP_INFO(node->get_logger(), "  point_B = %s", point_B.c_str());

  // ─────────────────────────────────────────────────
  // 2. LIFT CONTINUOUS PUBLISHER
  // ─────────────────────────────────────────────────
  // Sorun: JointGroupEffortController sürekli komut bekler. BT node'ları
  // arasında geçişte (örn. NavigateToPose çalışırken) kimse effort yayınlamaz
  // → yerçekimi lift'i aşağı çeker.
  //
  // Çözüm: shared_effort double'ını tüm LiftControl node'larıyla paylaş.
  // Bu timer her 100ms'de mevcut shared_effort değerini yayınlar.
  // LiftControl sadece bu değeri günceller — timer yayını sürdürür.
  auto shared_effort = std::make_shared<double>(0.0);

  auto lift_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/lift_controller/commands", 10);

  auto lift_timer = node->create_wall_timer(
    std::chrono::milliseconds(100),
    [lift_pub, shared_effort]() {
      std_msgs::msg::Float64MultiArray msg;
      msg.data = {*shared_effort};
      lift_pub->publish(msg);
    });

  // ─────────────────────────────────────────────────
  // 3. BT FACTORY
  // ─────────────────────────────────────────────────
  BT::BehaviorTreeFactory factory;

  // LiftControl: shared_effort pointer'ı lambda ile yakalanır
  factory.registerBuilder<LiftControl>(
    "LiftControl",
    [node, shared_effort](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<LiftControl>(name, config, node, shared_effort);
    });

  factory.registerBuilder<NavigateToPose>(
    "NavigateToPose",
    [node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<NavigateToPose>(name, config, node);
    });

  factory.registerBuilder<WaitSeconds>(
    "WaitSeconds",
    [node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<WaitSeconds>(name, config, node);
    });

  // ─────────────────────────────────────────────────
  // 4. BLACKBOARD — waypoint değerlerini yaz
  // ─────────────────────────────────────────────────
  auto blackboard = BT::Blackboard::create();
  blackboard->set("point_A", point_A);
  blackboard->set("point_B", point_B);

  // ─────────────────────────────────────────────────
  // 5. XML'DEN AĞACI OLUŞTUR
  // ─────────────────────────────────────────────────
  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(bt_xml_path, blackboard);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "BT XML yuklenemedi: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Behaviour Tree yuklendi. Gorev basliyor...");

  // ─────────────────────────────────────────────────
  // 6. TICK DÖNGÜSÜ
  // ─────────────────────────────────────────────────
  // spin_some(): timer callback dahil tüm pending callback'leri işler.
  //   → lift_timer her 100ms'de publish yapar (shared_effort güncel değeri)
  //   → NavigateToPose result_callback bu sayede işlenir
  // tickRoot(): BT'yi bir adım ilerletir

  rclcpp::Rate rate(10);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    rclcpp::spin_some(node);   // timer + action callback'leri işle
    status = tree.tickRoot();  // BT adımını çalıştır
    rate.sleep();
  }

  // ─────────────────────────────────────────────────
  // 7. SONUÇ — lift'i sıfırla
  // ─────────────────────────────────────────────────
  *shared_effort = 0.0;
  rclcpp::spin_some(node);  // son 0N komutunu gönder

  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(),  "=== GOREV TAMAMLANDI (SUCCESS) ===");
  } else {
    RCLCPP_ERROR(node->get_logger(), "=== GOREV BASARISIZ (FAILURE) ===");
  }

  rclcpp::shutdown();
  return (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
}
