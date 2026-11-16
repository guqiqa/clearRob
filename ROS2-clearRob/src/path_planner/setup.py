from setuptools import find_packages, setup

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AIview Dev',
    maintainer_email='dev@aiview.com',
    description='Global coverage path planning, Pure Pursuit control, local replanning, and TSP ordering for cleaning robots.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner_node = path_planner.planner_node:main',
        ],
    },
)
