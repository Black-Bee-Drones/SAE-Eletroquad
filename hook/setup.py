from setuptools import find_packages, setup

package_name = "hook"

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
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mangalarga = hook.mangalarga:main",
            "test_takeoff_states = hook.tests.test_takeoff_states:main",
            "test_follow_blue_line_pid = hook.tests.test_follow_blue_line_pid:main",
            "test_rtl = hook.tests.test_rtl:main",
        ],
    },
)
