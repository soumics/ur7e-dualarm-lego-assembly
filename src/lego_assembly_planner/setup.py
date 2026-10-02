from glob import glob

from setuptools import find_packages, setup

package_name = "lego_assembly_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/models", glob("models/*.ldr")),
        ("share/" + package_name + "/scripts", glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Soumic Sarkar",
    maintainer_email="soumic4it@gmail.com",
    description="LDraw target -> dependency graph -> feasible dual-arm assembly plan.",
    license="Proprietary",
)
