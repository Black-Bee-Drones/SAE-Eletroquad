from setuptools import find_packages, setup

package_name = "slalom"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="samuel",
    maintainer_email="samuellimabraz@gmail.com",
    description="Autonomous drone slalom mission package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "slalom_mission = slalom.slalom_mission:main",
            "black_line_detection_node = slalom.utils.black_line_detection_node:main",
        ],
    },
)
