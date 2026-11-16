from setuptools import find_packages, setup

package_name = 'motion_controller'

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
    maintainer_email='dev@aiview.tech',
    description='Motion actuator node for cleaning robot: 4-wheel differential drive, '
                'PID speed control, odometry, emergency stop, and slope assist.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_controller_node = motion_controller.motion_node:main',
        ],
    },
)
