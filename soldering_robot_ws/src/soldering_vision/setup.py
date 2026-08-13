from glob import glob
import os

from setuptools import find_packages, setup


package_name = "soldering_vision"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="phorce",
    maintainer_email="phorce@example.com",
    description="Planar workspace perception for the soldering robot.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vision_observer = "
            "soldering_vision.vision_observer_node:main",
            "synthetic_scene = "
            "soldering_vision.synthetic_scene_node:main",
            "train_convnext = "
            "soldering_vision.train_convnext:main",
            "build_process_crops = "
            "soldering_vision.build_process_crops:main",
        ],
    },
)
