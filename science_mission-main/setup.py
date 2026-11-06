from setuptools import find_packages, setup

package_name = "science_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/science_launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sheikh Shakib Hossain",
    maintainer_email="sheikhshakibpro@gmail.com",
    description="Science mission nodes",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_feed_node   = science_mission.camera_feed_node:main",
            "arm_node           = science_mission.arm_node:main",
            "bifrost_node       = science_mission.bifrost_node:main",
            "sensor_fusion_node = science_mission.sensor_fusion_node:main",
            "gps_imu_node = science_mission.gps_imu_node:main",
            "keypad_publisher = science_mission.numpad_publisher:main",
        ],
    },
)
