from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo del joystick
        Node(package='joy', executable='joy_node', output='screen'
        ),

        # Nodo para el control
        Node(
            package='robotic_arm_joy', executable='joint_controller',  output='screen'
        )
    ])
