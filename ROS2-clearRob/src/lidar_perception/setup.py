from setuptools import setup

package_name = 'lidar_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AIview Dev',
    maintainer_email='dev@aiview.com',
    description='LiDAR perception node for the cleaning robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_perception_node = lidar_perception.lidar_node:main',
        ],
    },
)
