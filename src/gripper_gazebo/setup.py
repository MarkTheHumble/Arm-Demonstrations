from glob import glob

from setuptools import find_packages, setup

package_name = "gripper_gazebo"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages/", glob(f"resource/{package_name}")),
        (f"share/{package_name}/", glob("package.xml")),
        (f"share/{package_name}/config/", glob(f"{package_name}/config/*")),
        (f"share/{package_name}/urdf/", glob(f"{package_name}/urdf/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pierce",
    maintainer_email="dev@piercecoyle.com",
    description="TODO: Package description",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={"console_scripts": []},
)
