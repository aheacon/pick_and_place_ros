from setuptools import find_packages, setup
from glob import glob
import os

package_name = "mycobot_description"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/urdf", glob("urdf/*")),
        ("share/" + package_name + "/launch", glob("launch/*")),
        ("share/" + package_name + "/rviz", glob("rviz/*")),
        ("share/" + package_name + "/mesh", glob("mesh/*")),
        ("share/" + package_name + "/worlds", glob("worlds/*")),
        ("share/" + package_name + "/xacro", glob("xacro/*")),
        ("share/" + package_name + "/config", glob("config/*")),
        # (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="anel",
    maintainer_email="anel@eacon.ba",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
