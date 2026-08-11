from setuptools import find_packages, setup
from glob import glob

package_name = 'ur5e_motion_planning_lab'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sid',
    maintainer_email='sid@example.com',
    description='MoveIt 2 motion planning experiments on UR5e (fake hardware)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ex1_joint_goal = ur5e_motion_planning_lab.ex1_joint_goal:main',
            'ex2_pose_goal = ur5e_motion_planning_lab.ex2_pose_goal:main',
            'ex3_cartesian_path = ur5e_motion_planning_lab.ex3_cartesian_path:main',
            'ex4_collision_objects = ur5e_motion_planning_lab.ex4_collision_objects:main',
            'ex5_planner_comparison = ur5e_motion_planning_lab.ex5_planner_comparison:main',
            'ex6_pilz_linear = ur5e_motion_planning_lab.ex6_pilz_linear:main',
            'ex7_orientation_constraints = ur5e_motion_planning_lab.ex7_orientation_constraints:main',
            'ex8_attach_detach_object = ur5e_motion_planning_lab.ex8_attach_detach_object:main',
        ],
    },
)
