from setuptools import find_packages, setup

package_name = 'fusion_engine'

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
    description='Sensor fusion node for cleaning robot: fuses vision 2D detections '
                'with LiDAR 3D point clouds to produce 3D target estimations '
                'and obstacle avoidance decisions.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_engine_node = fusion_engine.fusion_node:main',
        ],
    },
)
