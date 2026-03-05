from setuptools import setup, find_packages

package_name = 'ugv_base_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@todo.todo',
    description='Waveshare UGV ESP32 motor driver for ROS2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motor_driver_node = ugv_base_driver.motor_driver_node:main',
            'stub_motor_node = ugv_base_driver.stub_motor_node:main',
        ],
    },
)
