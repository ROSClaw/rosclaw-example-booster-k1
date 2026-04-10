from setuptools import setup


package_name = "k1_low_level_relay"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/k1_low_level_relay.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Codex",
    maintainer_email="codex@example.com",
    description="Robot-local relay for Booster K1 low-level DDS topics.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "k1_low_level_relay_node = k1_low_level_relay.relay_node:main",
        ],
    },
)
