from setuptools import setup

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AIview Dev',
    maintainer_email='aiview@example.com',
    description='Coverage path planner (boustrophedon) driving the V3 chassis via /chassis/intent',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'path_planner_node = path_planner.path_planner_node:main',
        ],
    },
)
