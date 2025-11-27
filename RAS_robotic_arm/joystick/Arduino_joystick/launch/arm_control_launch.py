import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Obtener ruta del paquete
    package_dir = get_package_share_directory('robotic_arm_joy')
    params_file = os.path.join(package_dir, 'config', 'params.yaml')
    
    return LaunchDescription([
        # Nodo Joy (leer joystick)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',  # Dispositivo joystick (modificar según tu sistema)
                'deadzone': 0.05,
                'autorepeat_rate': 0.0,
            }]
        ),
        
        # Nodo personalizado: Joystick → Serial → Arduino
        Node(
            package='robotic_arm_joy',
            executable='joystick_serial',
            name='joystick_serial_node',
            output='screen',
            parameters=[params_file]
        ),
    ])
