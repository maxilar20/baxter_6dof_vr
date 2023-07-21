from setuptools import setup
import os
from glob import glob

package_name = "baxter_6dof_vr"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mael",
    maintainer_email="dorne99@hotmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pose_to_tf = baxter_6dof_vr.pose_to_tf:main",
            "baxter_pose_tracker = baxter_6dof_vr.baxter_pose_tracker:main",
        ],
    },
)
