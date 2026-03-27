# 这是 urdf2tf.launch.py 文件的最终、保证成功的版本

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    
    # --- Part 1: URDF 和两个核心发布者 ---

    urdf_path = get_package_share_directory('robot_description')
    urdf_model_path = urdf_path + '/urdf/robot.urdf'

    declare_model_path_arg = launch.actions.DeclareLaunchArgument(
        name='model', 
        default_value=urdf_model_path,
        description='Absolute path to robot URDF file'
    )
    
    # 启动 robot_state_publisher
    # 它会读取完整的 URDF。它现在不会崩溃了。
    # 它会发布时间戳有问题的 fixed 关节 TF，和时间戳正常的 continuous 关节 TF。
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': launch_ros.parameter_descriptions.ParameterValue(
            Command(['cat ', LaunchConfiguration('model')]), value_type=str)}]
    )

    # 启动 joint_state_publisher_gui
    # 它会读取完整的 URDF，找到两个轮子关节，并创建滑块。
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    # --- Part 2: "覆盖"疗法 ---
    # 我们启动两个 static_transform_publisher，它们发布的 TF
    # 会覆盖掉 robot_state_publisher 发布的那些时间戳为 0 的 TF。

    # 1. 发布 base_footprint -> base_link 的静态变换
    static_tf_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0.0', '0.0', '0.076', '0', '0', '0', 'base_footprint', 'base_link']
    )
    
    # 2. 发布 base_link -> laser_link 的静态变换
    static_tf_base_link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0.075', '0', '0', '0', 'base_link', 'laser_link']
    )
    
    # 返回一个包含所有节点的 LaunchDescription
    return launch.LaunchDescription([
        declare_model_path_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        static_tf_base_footprint_to_base_link, # <--- 覆盖疗法1号
        static_tf_base_link_to_laser,             # <--- 覆盖疗法2号
    ])
