from glob import glob

from setuptools import setup

package_name = "ican_referee"

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
    zip_safe=False,
    maintainer="iCAN Rescue Team",
    maintainer_email="team@ican.local",
    description="iCAN 裁判通信模块（E-01）与接口一致性监控",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "referee_monitor_node = ican_referee.referee_monitor_node:main",
            "referee_selfcheck = ican_referee.interface_selfcheck:main",
        ],
    },
)
