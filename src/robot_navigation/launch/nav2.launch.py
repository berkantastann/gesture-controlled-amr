import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav_share     = get_package_share_directory('robot_navigation')
    nav2_bringup  = get_package_share_directory('nav2_bringup')
    bringup_share = get_package_share_directory('robot_bringup')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Gazebo simülasyon saatini kullan',
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav_share, 'maps', 'map_no_point.yaml'),
        description='Kullanılacak harita YAML dosyasının tam yolu',
    )
    params_file_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(nav_share, 'params', 'nav2_params.yaml'),
        description='Nav2 parametre dosyasının tam yolu',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Nav2 RViz arayüzünü başlat',
    )

    def launch_setup(context, *args, **kwargs):
        # global context'teki 'use_rviz'i eziyor; launch argümanlarından alıyor
        use_sim_time = context.launch_configurations.get('use_sim_time', 'true')
        use_rviz     = context.launch_configurations.get('use_rviz', 'true')
        map_yaml     = context.launch_configurations.get('map')
        params_file  = context.launch_configurations.get('nav2_params_file')

        bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'full.launch.py')
            ),
            launch_arguments={'use_rviz': 'false'}.items(),
        )

        localization_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'localization_launch.py')
            ),
            launch_arguments={
                'map':          map_yaml,
                'use_sim_time': use_sim_time,
                'params_file':  params_file,
                'autostart':    'true',
            }.items(),
        )

        # Navigasyon yığını: controller, planner, bt_navigator, velocity_smoother…
        # cmd_vel zinciri:
        #   controller_server → cmd_vel_nav → velocity_smoother → /cmd_vel
        #   → full.launch.py relay → /diff_drive_controller/cmd_vel_unstamped
        navigation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file':  params_file,
                'autostart':    'true',
            }.items(),
        )

        actions = [bringup_launch, localization_launch, navigation_launch]

        if use_rviz.lower() == 'true':
            actions.append(Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(nav_share, 'rviz', 'nav2.rviz')],
                parameters=[{'use_sim_time': use_sim_time == 'true'}],
            ))

        return actions

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        params_file_arg,
        use_rviz_arg,
        OpaqueFunction(function=launch_setup),
    ])
