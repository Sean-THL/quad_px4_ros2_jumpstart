from glob import glob
from setuptools import setup


setup(
    name='trajectory_follower',
    version='0.1.0',
    packages=['trajectory_follower'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/trajectory_follower']),
        ('share/trajectory_follower', ['package.xml']),
        ('share/trajectory_follower/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='admin@todo.todo',
    description='Live analytic and CSV PX4 Offboard trajectory publishers',
    license='MIT',
    entry_points={
        'console_scripts': [
            'follow_csv = trajectory_follower.follow_csv:main',
            'follow_live = trajectory_follower.follow_live:main',
            'teleop_keyboard = trajectory_follower.teleop_keyboard:main',
        ],
    },
)
