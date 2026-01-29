from setuptools import setup

package_name = "security_patrol"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Duckie02 Security Patrol",
    license="MIT",
    entry_points={
        "console_scripts": [
            "security_patrol_node = security_patrol.main:main",
            "image_saver = security_patrol.image_saver:main",
        ],
    },
)
