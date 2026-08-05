from setuptools import setup

package_name = 'fusion_engine'

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
    description='Semantic fusion engine (Phase 2 skeleton) — alert gateway for master state machine',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fusion_node = fusion_engine.fusion_node:main',
        ],
    },
)
