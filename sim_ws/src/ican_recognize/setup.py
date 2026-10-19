from glob import glob
from setuptools import setup

package_name = "ican_recognize"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="iCAN User",
    maintainer_email="user@example.com",
    description="Ground target recognition node for the iCAN rescue robot challenge.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "recognize_node = ican_recognize.recognize_node:main",
        ],
    },
)
