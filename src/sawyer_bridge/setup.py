from glob import glob
from setuptools import setup

setup(name='sawyer_bridge', version='0.3.0', packages=['sawyer_bridge'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/sawyer_bridge']),
                  ('share/sawyer_bridge', ['package.xml']),
                  ('share/sawyer_bridge/launch', glob('launch/*.launch.py')),
                  ('share/sawyer_bridge/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Sawyer Bridge contributors', maintainer_email='maintainers@example.com',
      description='Launch and configure the Sawyer bridge for system Humble', license='MIT',
      entry_points={'console_scripts': [
          'configure_bridge = sawyer_bridge.configure:main',
          'check = sawyer_bridge.check:main']})
