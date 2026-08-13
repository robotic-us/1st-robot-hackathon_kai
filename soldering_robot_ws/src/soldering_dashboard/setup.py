from glob import glob
from setuptools import find_packages, setup


package_name = "soldering_dashboard"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="phorce",
    maintainer_email="phorce@example.com",
    description="Soldering robot operations dashboard.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dashboard = soldering_dashboard.dashboard_app:main",
        ],
    },
)
