from setuptools import find_packages, setup

package_name = 'master_controller'

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
    maintainer_email='dev@aiview.tech',
    description='Master controller for the cleaning robot system.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'master_controller_node = master_controller.master_node:main',
        ],
    },
)
