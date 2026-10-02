from setuptools import find_packages, setup

package_name = "lego_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/perception.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Soumic Sarkar",
    maintainer_email="soumic4it@gmail.com",
    description="LEGO brick perception: ground-truth backend, later RGB-D detection.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "ground_truth_detector = lego_perception.ground_truth_detector:main",
            "detection_markers = lego_perception.detection_markers:main",
        ],
    },
)
