cmake_minimum_required(VERSION 3.8)
project(robot_mission_control)

# Derleyici uyarıları (iyi pratik)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Bağımlılıkları bul ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Ortak bağımlılık listesi — ament_target_dependencies'e geçilir
set(DEPENDENCIES
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  nav2_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
)

# include/ klasörü derleyiciye tanıtılır
include_directories(include)

# --- Executable: mission_executor_node ---
# Tüm .cpp dosyaları tek executable'da birleşir
add_executable(mission_executor_node
  src/lift_control_action.cpp
  src/navigate_to_pose_action.cpp
  src/wait_seconds_action.cpp
  src/mission_executor_node.cpp
)
ament_target_dependencies(mission_executor_node ${DEPENDENCIES})

# --- Kurulum ---
install(TARGETS mission_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

# launch/, config/, behavior_trees/ share/ altına kopyalanır
# launch dosyasını ros2 launch ile bulmak için zorunlu
install(DIRECTORY
  launch
  config
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
