from setuptools import setup

package_name = "ros2monitor"

setup(
    name=package_name,
    version="0.0.1",
    package_dir={"": "src"},
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jiaqi Li",
    maintainer_email="ljq0831@qq.com",
    description="A TUI version of rqt_topic",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ros2monitor = " + package_name + ".screen:main",
        ],
    },
)
