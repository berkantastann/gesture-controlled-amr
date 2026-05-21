from setuptools import find_packages, setup

package_name = 'robot_gesture_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['hand_landmarker.task']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berkantastan',
    maintainer_email='btastan9@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gesture_control_node = robot_gesture_control.hands_control_node:main',
        ],
    },
)
