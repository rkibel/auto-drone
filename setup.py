from setuptools import find_packages, setup


package_name = "auto_drone"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=("auto_drone", "auto_drone.*")),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["numpy", "setuptools"],
    zip_safe=True,
    maintainer="ron",
    maintainer_email="ron@example.com",
    description="Tree-focused PX4/Gazebo autonomy demonstration.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "px4_control_node = auto_drone.px4_control_node:main",
        ],
    },
)
