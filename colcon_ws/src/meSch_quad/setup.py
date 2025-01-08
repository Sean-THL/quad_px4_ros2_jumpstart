from setuptools import find_packages, setup

package_name = 'meSch_quad'

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
    description='meSch',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'meSch_CandTraj_node = meSch_quad.meSchCandTraj:main',
            'meSch_SpPub_node = meSch_quad.meSchSpPub:main'
        ],
    },
)
