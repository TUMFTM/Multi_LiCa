import os
from glob import glob

from setuptools import setup

package_name = "multi_lidar_calibrator"
submodules = "multi_lidar_calibrator/calibration"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name, submodules],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Andrii Chumak, Dominik Kulmer",
    maintainer_email="ge65wex@mytum.de, dominik.kulmer@tum.de",
    description="Multi - LiDAR-to-LiDAR calibration framework for ROS 2 and non-ROS applications",
    license="LGPL-3.0 license",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "multi_lidar_calibrator = multi_lidar_calibrator.multi_lidar_calibrator:main",
        ],
    },
)
