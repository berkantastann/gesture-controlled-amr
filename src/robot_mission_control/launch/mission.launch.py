import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mission_pkg = get_package_share_directory('robot_mission_control')
    nav_pkg     = get_package_share_directory('robot_navigation')

    bt_xml    = os.path.join(mission_pkg, 'behavior_trees', 'deliver_payload.xml')
    waypoints = os.path.join(mission_pkg, 'config', 'waypoints.yaml')

    # ─────────────────────────────────────────────────────────
    # Launch argümanları
    # Kullanım: ros2 launch robot_mission_control mission.launch.py use_rviz:=false
    # ─────────────────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Nav2 RViz arayuzunu baslat',
    )
    use_rviz = LaunchConfiguration('use_rviz')

    # ─────────────────────────────────────────────────────────
    # 1. Nav2 stack — Gazebo + AMCL + Nav2 + isteğe bağlı RViz
    #    full.launch.py → localization → navigation hepsini kapsar
    # ─────────────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'use_rviz': use_rviz,
        }.items(),
    )

    # ─────────────────────────────────────────────────────────
    # 2. Mission Executor
    #    parameters listesinde iki kaynak var:
    #      a) waypoints.yaml → point_A, point_B ROS parametrelerini yükler
    #      b) sözlük        → bt_xml_path'i launch-time'da hesaplanmış
    #                          tam yol ile set eder
    # ─────────────────────────────────────────────────────────
    mission_executor = Node(
        package='robot_mission_control',
        executable='mission_executor_node',
        name='mission_executor',
        parameters=[
            waypoints,
            {'bt_xml_path': bt_xml},
        ],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────
    # 3. TimerAction: mission executor'ı geciktir
    #    Nav2 tam ayağa kalkmadan goal gönderilirse action server
    #    bulunamaz hatası alınır.
    #    Gazebo + controller spawn + AMCL + Nav2 toplam ~15-20s
    # ─────────────────────────────────────────────────────────
    delayed_executor = TimerAction(
        period=30.0,  # saniye — Gazebo + AMCL + Nav2 tam aktif olana kadar bekle
        actions=[mission_executor],
    )

    return LaunchDescription([
        use_rviz_arg,
        nav2_launch,
        delayed_executor,
    ])
