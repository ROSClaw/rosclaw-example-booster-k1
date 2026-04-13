from setuptools import setup


package_name = "k1_visionos_rtabmap_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/k1_visionos_rtabmap_bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Codex",
    maintainer_email="codex@example.com",
    description="Collect headset spatial uploads for the K1 visionOS frontend and publish aggregate mapping status.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "k1_visionos_rtabmap_bridge_node = k1_visionos_rtabmap_bridge.mapping_bridge:main",
        ],
    },
)
