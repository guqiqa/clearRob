from setuptools import find_packages, setup

package_name = 'control_gateway'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AIview Dev',
    maintainer_email='dev@aiview.local',
    description='System entry point - receives raw commands, translates to UnifiedCommand, handles priority arbitration, and publishes motion velocity in MANUAL mode',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_gateway_node = control_gateway.gateway_node:main',
        ],
    },
)
