from glob import glob

from setuptools import setup

package_name = "ican_shoot"

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
    description="iCAN 陆地清障射击模块（环形靶瞄准、靶位瞄准、射击闭环）",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ring_shooter_node = ican_shoot.ring_shooter_node:main",
        ],
    },
)
