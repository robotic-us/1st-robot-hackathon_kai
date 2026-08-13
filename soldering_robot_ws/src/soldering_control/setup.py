from setuptools import find_packages, setup


package_name = "soldering_control"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/six_motor_practice.launch.py",
                "launch/two_motor_hardware_setup.launch.py",
                "launch/two_motor_physical_observation.launch.py",
                "launch/pcm_session_daemon.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="phorce",
    maintainer_email="phorce@example.com",
    description="Automatic soldering control practice nodes.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "auto_motor_nodes = soldering_control.auto_motor_nodes:main",
            "master_mcu_node = soldering_control.master_mcu_node:main",
            "hardware_setup_node = "
            "soldering_control.hardware_setup_node:main",
            "motor_axis_node = soldering_control.motor_axis_node:main",
            "physical_motor_node = "
            "soldering_control.physical_motor_node:main",
            "pcm_studio = soldering_control.pcm_studio_client:main",
            "pcm_relative_teaching = "
            "soldering_control.relative_teaching:main",
            "pcm_session_daemon = "
            "soldering_control.pcm_session_daemon:main",
            "play_motion_4_5 = "
            "soldering_control.play_motion_4_5:main",
            "run_motion_4_5 = "
            "soldering_control.run_motion_4_5:main",
            "pvector_sim_practice = "
            "soldering_control.pvector_sim_practice:main",
            "six_motor_coordinator = "
            "soldering_control.six_motor_coordinator:main",
            "two_axis_practice = soldering_control.two_axis_practice:main",
        ],
    },
)
