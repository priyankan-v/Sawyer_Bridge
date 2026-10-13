from setuptools import setup
from glob import glob

setup(name='sawyer_ros2_control', version='0.1.0', packages=['sawyer_ros2_control'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/sawyer_ros2_control']),
                  ('share/sawyer_ros2_control', ['package.xml', 'joint_limits.json']),
                  ('share/sawyer_ros2_control/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Sawyer workstation maintainers', maintainer_email='maintainer@example.com',
      description='Guarded Sawyer control and read-only camera viewing through ros1_bridge', license='Apache-2.0',
      entry_points={'console_scripts': ['command = sawyer_ros2_control.command:main',
                                        'move_joint = sawyer_ros2_control.move_joint:main',
                                        'sawyer_runtime = sawyer_ros2_control.runtime:main',
                                        'preflight = sawyer_ros2_control.runtime:preflight_main',
                                        'verify_bridge = sawyer_ros2_control.runtime:verify_main',
                                        'camera_viewer = sawyer_ros2_control.camera_viewer:main']})
