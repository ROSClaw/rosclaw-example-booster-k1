from setuptools import setup


package_name = "k1_cmd_vel_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/k1_cmd_vel_bridge.launch.py",
                "launch/k1_openclaw_bringup.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Codex",
    maintainer_email="codex@example.com",
    description="Bridge ROS 2 Twist commands to Booster K1 locomotion RPC calls.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "k1_cmd_vel_bridge_node = k1_cmd_vel_bridge.cmd_vel_bridge:main",
        ],
    },
)
