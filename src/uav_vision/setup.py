from setuptools import setup, find_packages
from glob import glob

package_name = 'uav_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/uav_vision_all.launch.py']),
        ('share/' + package_name + '/launch', ['launch/uav_vision_terminals.launch.py']),
        ('share/' + package_name + '/models', glob('models/*.pt')),
        ('share/' + package_name + '/models', glob('models/*.pb')),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Karol',
    maintainer_email='kp306682@studen.polsl.pl',
    description='UAV vision nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_stop_node = uav_vision.depth_stop_node:main',
            'disparity = uav_vision.disparity:main',
            'stop_controller_node = uav_vision.stop_controller_node:main',
            'uav_camera_det = uav_vision.uav_camera_det:main',
        ],
    },
)
