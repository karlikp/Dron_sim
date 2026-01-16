from setuptools import find_packages, setup
from glob import glob

package_name = 'uav_geotag_recorder'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karol',
    maintainer_email='pitera.karol@gmail.com',
    description='UAV sensors utilities (geotagging, etc.)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
             'geotag_recorder = uav_geotag_recorder.geotag_recorder:main',
        ],
    },
)
