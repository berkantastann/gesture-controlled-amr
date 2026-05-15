import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_share  = get_package_share_directory('robot_navigation')
    desc_share = get_package_share_directory('robot_description')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Gazebo simülasyon saatini kullan',
    )

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(nav_share, 'params', 'slam_toolbox_params.yaml'),
        description='SLAM Toolbox parametre dosyası',
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='RViz2 başlat',
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(nav_share, 'rviz', 'mapping.rviz'),
        description='RViz2 konfigürasyon dosyası',
    )

    use_sim_time    = LaunchConfiguration('use_sim_time')
    slam_params     = LaunchConfiguration('slam_params_file')
    use_rviz        = LaunchConfiguration('use_rviz')
    rviz_config     = LaunchConfiguration('rviz_config')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_bringup'),
                'launch', 'full.launch.py',
            )
        ),
    )


    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    # bringupda mevcut burada eklemedim 
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_params_file_arg,
        use_rviz_arg,
        rviz_config_arg,
        bringup_launch,
        slam_toolbox_node,
    ])
