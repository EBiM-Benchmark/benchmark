from setuptools import find_packages, setup

package_name = "keyboard_teleop_publisher"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="2houyuhang",
    maintainer_email="cahgzt@gmail.com",
    description="Full arm+base keyboard teleop publisher for the EBiM task-1 MuJoCo simulator",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "keyboard_teleop_publisher = keyboard_teleop_publisher.keyboard_teleop_publisher:main",
        ],
    },
)
