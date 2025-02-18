from setuptools import setup

package_name = "panda_iwt_ros2_plc_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="admin",
    maintainer_email="hazem.mt.youssef@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "plc_controller_node = panda_iwt_ros2_plc_controller.plc_controller_node:main",
        ],
    },
)
