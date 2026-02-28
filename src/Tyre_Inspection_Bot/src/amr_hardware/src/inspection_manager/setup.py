from setuptools import setup, find_packages

package_name = "inspection_manager"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/inspection_manager.launch.py"]),
        ("share/" + package_name + "/launch", ["launch/inspection_mission_launch.py"]),
        ("share/" + package_name + "/config", ["config/trucks.yaml"]),
    ],
    install_requires=["setuptools", "pyyaml", "ament_index_python"],
    zip_safe=True,
    maintainer="inspection_manager",
    maintainer_email="user@example.com",
    description="Mission state machine for autonomous vehicle tire inspection using Aurora SLAM, Nav2, and 3D detections. Supports dynamic vehicle detection (car/truck) and tire inspection.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "inspection_manager_node = inspection_manager.inspection_manager_node:main",
            "photo_capture_service = inspection_manager.photo_capture_service:main",
            "manager_node = inspection_manager.manager_node:main",
            "photo_capture_api = inspection_manager.photo_capture_api:main",
            "visual_servo_align_server = inspection_manager.alignment:main",
        ],
    },
)

