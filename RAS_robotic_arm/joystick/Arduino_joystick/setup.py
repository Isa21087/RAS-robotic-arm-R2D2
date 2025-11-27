from setuptools import setup



package_name = 'robotic_arm_joy'



setup(

    name=package_name,

    version='0.0.0',

    packages=[package_name],

    data_files=[

        ('share/ament_index/resource_index/packages',

         ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='tu_nombre',

    maintainer_email='tu_correo@example.com',

    description='Control del brazo robótico con joystick en ROS 2',

    license='Apache License 2.0',

    tests_require=['pytest'],

    entry_points={

        'console_scripts': [

            'robotic_arm_joy = robotic_arm_joy.robotic_arm_joy:main',

        ],

    },

)
