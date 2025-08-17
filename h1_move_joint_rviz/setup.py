from setuptools import setup

package_name = 'h1_move_joint_rviz'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='banana-killer',
    maintainer_email='sashagrachev2005@gmail.com',
    description='ROS 2 nodes convert the joint position data of the Unitree H1\
        robot from the "iron" format (LowState, MotorStates) to the standard\
        JointState for visualization in RViz.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"move_joint_rviz_without_real_robot_node = {package_name}.move_joint_rviz_without_real_robot_node:main",
            f"move_joint_rviz_with_real_robot_node = {package_name}.move_joint_rviz_with_real_robot_node:main",
        ],
    },
)
