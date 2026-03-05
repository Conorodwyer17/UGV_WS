from setuptools import setup, find_packages

package_name = "aurora_mock"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/aurora_mock.launch.py"]),
        ("share/" + package_name + "/config", [
            "config/sensor_realism.yaml",
            "config/sensor_timing_realistic.yaml",
        ]),
    ],
    install_requires=["setuptools", "ament_index_python", "numpy"],
    zip_safe=True,
    maintainer="UGV",
    maintainer_email="user@example.com",
    description="Synthetic Aurora node for simulation without hardware",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "aurora_mock_node = aurora_mock.aurora_mock_node:main",
            "depth_generator_node = aurora_mock.depth_generator_node:main",
            "synthetic_vehicle_publisher = aurora_mock.synthetic_vehicle_publisher:main",
            "clock_publisher = aurora_mock.clock_publisher:main",
        ],
    },
)
