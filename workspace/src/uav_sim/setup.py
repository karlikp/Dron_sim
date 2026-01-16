from setuptools import find_packages, setup
from importlib import resources
from pathlib import Path
from glob import glob
import os

package_name = "uav_sim"
share_path = Path("share") / package_name

def get_data_files(directory):
    data_files = []
    for root, _, files in os.walk(directory):
        for file in files:
            src_path = os.path.join(root, file)
            dest_path = os.path.join(share_path.as_posix(), root)
            data_files.append((dest_path, [src_path]))
    return data_files

launch_files = [str(path) for path in Path("launch").glob("*.launch.py")]

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (share_path.as_posix(), ["package.xml"]),
        ((share_path / "launch").as_posix(), launch_files),
        *get_data_files('worlds'),
        *get_data_files('models'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karol',
    maintainer_email='pitera.karol@gmail.com',
    description='Bringup for UAV',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
