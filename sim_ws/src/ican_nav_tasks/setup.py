from glob import glob
from setuptools import setup

package_name = "ican_nav_tasks"

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
    description="iCAN 陆地清障导航模块节点",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "speed_monitor_node = ican_nav_tasks.speed_monitor_node:main",
            "sequential_mission_node = ican_nav_tasks.sequential_mission_node:main",
            "localization_eval_node = ican_nav_tasks.localization_eval_node:main",
        ],
    },
)
