from setuptools import setup

package_name = "indy7_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SirLab",
    maintainer_email="sirlab@todo.todo",
    description="Xbox controller teleoperation for Indy7 and Mark7 gripper.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "xbox_servo_node = indy7_teleop.xbox_servo_node:main",
        ],
    },
)
