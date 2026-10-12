from glob import glob

from setuptools import find_packages, setup

package_name = "rps"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages/", glob(f"resource/{package_name}")),
        (f"share/{package_name}/", glob("package.xml")),
        (f"share/{package_name}/", glob(f"{package_name}/*.py")),
        (f"share/{package_name}/launch/", glob(f"{package_name}/launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="morfinm3",
    maintainer_email="morifnm3@wwu.edu",
    description="rock, paper, scissors",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "rps = rps.rps:main",
        ],
    },
)
