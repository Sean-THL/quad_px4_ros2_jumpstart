from setuptools import find_packages, setup

package_name = 'eware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kalebbennaveed',
    maintainer_email='kalebbennaveed@gmail.com',
    description='Eware',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eware_node = eware.eware:main',
            'eware_CandTraj_node = eware.EwareCandTraj:main',
            'eware_SpPub_node = eware.EwareSpPub:main'
        ],
    },
)
