import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    fishbot_bringup_dir = get_package_share_directory(
        'bring_up')
    ydlidar_ros2_dir = get_package_share_directory(
        'ydlidar')
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_tcp_server = LaunchConfiguration('enable_tcp_server')

    urdf2tf = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [fishbot_bringup_dir, '/launch', '/urdf2tf.launch.py']),
    )

    ekf = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[fishbot_bringup_dir + '/config/ekf.yaml']
    )

    odom_filtered_to_tf = launch_ros.actions.Node(
        package='bring_up',
        executable='odom2tf',
        parameters=[{'odom_topic': '/odometry/filtered'}],
        output='screen'
    )

    microros_agent = launch_ros.actions.Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['udp4','--port','8888'],
        output='screen'
    )

    ros_serail2wifi =  launch_ros.actions.Node(
        package='ros_serial2wifi',
        executable='tcp_server',
        parameters=[{'serial_port': '/tmp/tty_laser'}],
        condition=IfCondition(enable_tcp_server),
        output='screen'
    )

    ydlidar = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ydlidar_ros2_dir, '/launch', '/ydlidar_launch.py']),
        condition=IfCondition(enable_lidar),
    )

    # 使用 TimerAction 启动后 5 秒执行 ydlidar 节点
    ydlidar_delay = launch.actions.TimerAction(period=5.0, actions=[ydlidar])
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'enable_lidar',
            default_value='false',
            description='Start the lidar driver'),
        launch.actions.DeclareLaunchArgument(
            'enable_tcp_server',
            default_value='false',
            description='Start ros_serial2wifi tcp_server'),
        urdf2tf,
        microros_agent,
        ros_serail2wifi,
        ekf,
        odom_filtered_to_tf,
        ydlidar_delay
    ])
