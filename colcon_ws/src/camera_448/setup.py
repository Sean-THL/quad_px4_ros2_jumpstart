from glob import glob
from setuptools import find_packages, setup

package_name = 'camera_448'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Publish calibrated 448x448 RGB-D crops from a RealSense D455.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'center_crop = camera_448.center_crop:main',
            'validate_flight_bag = camera_448.validate_bag:main',
            'visualize_flight_bag = camera_448.visualize_bag:main',
        ],
    },
)
