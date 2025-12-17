from setuptools import find_packages, setup

package_name = 'thermal_tc001'

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
    maintainer='Yixiang Gao',
    maintainer_email='ygao@mst.edu',
    description='ROS 2 Node for capturing and processing TC001 thermal camera data using OpenCV.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thermal_node = thermal_tc001.thermal_node:main',
        ],
    },
)