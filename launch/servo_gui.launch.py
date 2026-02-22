from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('servo')
    urdf_file = os.path.join(pkg, 'urdf', 'servo.urdf')
    rviz_config = os.path.join(pkg, 'rviz', 'servo.rviz')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    welcome_msg = LogInfo(msg="""
=========================================================
  ____  _____ ____  __     __ ___  
 / ___|| ____|  _ \ \ \   / // _ \ 
 \___ \|  _| | |_) | \ \ / /| | | |
  ___) | |___|  _ <   \ V / | |_| |
 |____/|_____|_| \_\   \_/   \___/ 
                                        
 Starting servo control and visualization with rviz        
 By Dilip Kumar
=========================================================
""")

    return LaunchDescription([
        welcome_msg,
        # Publishes URDF to /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Unified GUI: joint slider + action buttons in one window
        Node(
            package='servo',
            executable='servo_panel',
        ),

        # RViz2 showing the servo URDF model
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),

        # Serial bridge → Arduino
        Node(
            package='servo',
            executable='servo_node',
        ),
    ])
