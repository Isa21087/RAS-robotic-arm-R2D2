from setuptools import setup
import os
from glob import glob

package_name = 'robotic_arm_joy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch','*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marlon',
    maintainer_email='marlongarcia5465@gmail.com',
    description='Control del brazo robótico con joystick en ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_controller = robotic_arm_joy.robotic_arm_joy:main',
        ],
    },
)
