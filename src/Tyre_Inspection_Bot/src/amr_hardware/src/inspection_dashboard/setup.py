from setuptools import setup, find_packages

package_name = "inspection_dashboard"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/inspection_dashboard.launch.py"]),
        ("share/" + package_name + "/config", ["config/dashboard.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="inspection_dashboard",
    maintainer_email="user@example.com",
    description="Web dashboard for inspection state, diagnostics, and mission report observability.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "inspection_dashboard_node = inspection_dashboard.inspection_dashboard_node:main",
        ],
    },
)
