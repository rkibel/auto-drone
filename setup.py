from setuptools import find_packages, setup

package_name = "auto_drone"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/gazebo_lidar_bridge.launch.py",
                "launch/gazebo_lidar_demo.launch.py",
            ],
        ),
        (
            f"share/{package_name}/worlds",
            [
                "worlds/drone_reconstruction_world.sdf",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ron",
    maintainer_email="ron@example.com",
    description="Headless autonomous drone exploration with active mapping and uncertainty-aware planning.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "headless_3d_runner = auto_drone.headless_3d_runner:main",
            "gazebo_lidar_bridge = auto_drone.gazebo_lidar_bridge:main",
        ],
    },
)
