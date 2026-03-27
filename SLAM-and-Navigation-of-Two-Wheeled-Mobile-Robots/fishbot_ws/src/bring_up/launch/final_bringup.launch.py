# 这是 bringup_WITHOUT_LIDAR.launch.py 的内容
# 我们的目标是验证一个没有雷达的 TF 树是否健康

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    fishbot_bringup_share = get_package_share_directory('fishbot_bringup')
    enable_tcp_server = LaunchConfiguration('enable_tcp_server')

    # 启动 micro-ROS Agent
    microros_agent = launch_ros.actions.Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['udp4', '--port', '8888'],
        output='screen'
    )

    # 启动 odom -> base_footprint 的发布者
    ekf = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[fishbot_bringup_share + '/config/ekf.yaml']
    )

    odom2tf = launch_ros.actions.Node(
        package='fishbot_bringup',
        executable='odom2tf',
        parameters=[{'odom_topic': '/odometry/filtered'}],
        output='screen'
    )

    # 启动 TCP Server (为了让ESP32正常工作，可能需要)
    ros_serail2wifi = launch_ros.actions.Node(
        package='ros_serial2wifi',
        executable='tcp_server',
        parameters=[{'serial_port': '/tmp/tty_laser'}],
        condition=IfCondition(enable_tcp_server),
        output='screen'
    )

    # 手动构建健康的静态 TF 部分
    tf_base_footprint_to_base_link = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.076', '0', '0', '0', 'base_footprint', 'base_link']
    )

    tf_base_link_to_laser = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.075', '0', '0', '0', 'base_link', 'laser_link']
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'enable_tcp_server',
            default_value='false',
            description='Start ros_serial2wifi tcp_server'),
        microros_agent,
        ros_serail2wifi,
        ekf,
        odom2tf,
        tf_base_footprint_to_base_link,
        tf_base_link_to_laser
    ])
