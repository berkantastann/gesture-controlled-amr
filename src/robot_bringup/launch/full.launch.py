import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    use_rviz = LaunchConfiguration('use_rviz')

    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    world_file_path = os.path.join(pkg_share, 'worlds', 'empty0.sdf')

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file_path}'}.items(),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )
    
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'amr',
                   '-x', '0.0', '-z', '0.0'],
        output='screen',
    )


    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    lift_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['lift_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Sim saati
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Sensörler
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/rgb_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Ground truth odom: Ignition OdometryPublisher plugin -> /ground_truth/odom
    # Plugin'in TF'i Gazebo dahili kalır (ros_gz_bridge'de bridge edilmez); ROS /tf temiz.
    # header.frame_id = ground_truth_odom (diff_drive 'odom'undan ayrı, çakışma yok)
    ground_truth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/amr/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[('/model/amr/odometry', '/ground_truth/odom')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Static TF: odom -> ground_truth_odom (identity).
    # Amaç: RViz'de fixed_frame=odom seçildiğinde ground_truth/odom Odometry
    # display'i de aynı pencerede görünsün ve diff_drive/EKF ile birebir karşılaştırılabilsin.
    # Robot (0,0,0)'da spawn edildiği için iki frame başlangıçta üst üste; sapma = enkoder/EKF hatası.
    ground_truth_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_ground_truth_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'ground_truth_odom'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    after_spawn_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    after_jsb_diff_and_lift = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner, lift_spawner],
        )
    )
    
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'base.rviz')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch RViz2 with base config'),
        ign_gazebo,
        robot_state_publisher_node,
        spawn_entity,
        bridge,
        ground_truth_bridge,
        ground_truth_static_tf,
        cmd_vel_relay,
        after_spawn_jsb,
        after_jsb_diff_and_lift,
        rviz_node,
    ])
