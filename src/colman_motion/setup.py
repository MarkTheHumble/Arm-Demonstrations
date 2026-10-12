from glob import glob

from setuptools import find_packages, setup

package_name = "colman_motion"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages/", glob(f"resource/{package_name}")),
        (f"share/{package_name}/", glob("package.xml")),
        (f"share/{package_name}/launch/", glob(f"{package_name}/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pierce",
    maintainer_email="dev@piercecoyle.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "pick_and_place = colman_motion.pick_and_place:main",
            "eye_in_hand = colman_motion.eye_in_hand:main",
        ],
    },
)
