from setuptools import find_packages, setup

package_name = "art_dtrack_ros"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Fabian Wieczorek",
    maintainer_email="f.wieczorek@dlr.de",
    description="Reads and parses UDP messages from ART DTRACK systems",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": ["art_dtrack_tf = art_dtrack_ros.art_dtrack_tf:main"],
    },
)
