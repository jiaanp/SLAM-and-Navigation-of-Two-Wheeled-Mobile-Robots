# healthy_tf_for_slam.launch.py
# 目标：为 SLAM 提供一个完美的、不依赖 robot_state_publisher 的 TF 树

import launch
from launch_ros.actions import Node

def generate_launch_description():

    # 我们将手动定义所有从 odom 开始的变换
    
    # 1. odom -> base_footprint
    # 这个由您的 odom2tf 节点负责，所以这里我们不需要启动
    # 如果 odom2tf 没有运行，可以取消下面这段的注释来临时替代
    # tf_odom_to_base = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = ['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    # )

    # 2. base_footprint -> base_link
    tf_base_to_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0.0', '0.0', '0.076', '0', '0', '0', 'base_footprint', 'base_link']
    )
    
    # 3. base_link -> laser_link
    tf_link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0.075', '0', '0', '0', 'base_link', 'laser_link']
    )
    
    # 返回一个只包含这些必要静态发布者的 LaunchDescription
    return launch.LaunchDescription([
        # tf_odom_to_base, # 暂时注释
        tf_base_to_link,
        tf_link_to_laser,
    ])