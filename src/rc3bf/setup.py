from setuptools import find_packages, setup

package_name = "rc3bf"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/two_robots_straight_line.launch.py"],
        ),
    ],
    install_requires=[
        "setuptools",
        "pytransform3d",
    ],
    zip_safe=True,
    maintainer="maciej",
    maintainer_email="rozewicz.maciej@gmail.com",
    description="Implements with RC3BF Control",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["robot = rc3bf.robot:main"],
    },
)
