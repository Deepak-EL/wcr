from glob import glob

from setuptools import setup

package_name = 'wcr_webots'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/resource', glob('resource/*.urdf') + glob('resource/*.rviz') + glob('resource/*.json')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': ['inspection_detector = wcr_webots.inspection_detector:main',
                                      'wall_frames = wcr_webots.wall_frames:main',
                                      'coverage_mission = wcr_webots.coverage_mission:main']},
    zip_safe=True,
    maintainer='Girish',
    maintainer_email='s.tanushreeskandha@gmail.com',
    description='WCR in Webots with ROS 2: driver plugin (adhesion, 4WIS, odometry), sensors, RViz',
    license='Apache-2.0',
)
