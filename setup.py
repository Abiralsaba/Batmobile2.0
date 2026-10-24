from glob import glob
from setuptools import setup

setup(
    name='mars_rover',
    version='0.2.0',
    packages=['rover'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mars_rover']),
        ('share/mars_rover', ['package.xml', 'config.yaml', 'input.json']),
        ('share/mars_rover/launch', glob('launch/*.launch.py')),
        ('share/mars_rover/web', ['web/index.html']),
    ],
    install_requires=['setuptools', 'fastapi>=0.115,<1.0', 'uvicorn>=0.30,<1.0'],
    zip_safe=True,
    maintainer='Rover Team',
    maintainer_email='team@example.invalid',
    description='Non-actuating astronaut autonomy starter for ROS 2 Humble',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'mission_control_node = rover.mission_control_node:main',
        'mission_dashboard = rover.web:main',
        'rover_demo = rover.demo:main',
        'zed_body_bridge = rover.zed_body_bridge:main',
    ]},
)
