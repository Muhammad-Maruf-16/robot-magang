from glob import glob

from setuptools import setup

package_name = "deteksilokasi"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="ahmaddzikrullahfreedomjayakusu@gmail.com",
    description="Robot menentukan sendiri posisinya di peta (global localization dari scan LiDAR).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "deteksi_lokasi_node = deteksilokasi.deteksi_lokasi_node:main",
        ],
    },
)
