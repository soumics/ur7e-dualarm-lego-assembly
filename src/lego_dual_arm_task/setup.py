from setuptools import find_packages, setup

package_name = "lego_dual_arm_task"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Soumic Sarkar",
    maintainer_email="soumic4it@gmail.com",
    description="Executes dual-arm LEGO assembly plans: per-arm workers and the shared-zone arbiter.",
    license="Proprietary",
    entry_points={"console_scripts": ["build_executor = lego_dual_arm_task.build_executor:main"]},
)
