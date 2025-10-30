from setuptools import find_packages, setup
from importlib import resources
from pathlib import Path
from glob import glob
import os

package_name = "sim_bringup"
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
    include_package_data=True,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="docker",
    maintainer_email="kp306682@student.polsl@gmail.com",
    description="Bringup package with launch files to run simulation",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
