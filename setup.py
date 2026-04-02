from glob import glob
from setuptools import find_packages, setup

package_name = 'h12_safety_layer'

# with open('requirements.txt') as f:
#     requirements = f.read().splitlines()

data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/config', glob('config/*.yaml')),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'], # + requirements,
    data_files=data_files,
    zip_safe=True,
    maintainer='tonyzyt2000',
    maintainer_email='zhangyt2000@gmail.com',
    description='ROS2 package for safety layer of h12',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'safety_layer_node = h12_safety_layer.ros2.safety_layer_node:main',
        ],
    },
)
