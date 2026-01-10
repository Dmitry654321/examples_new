from setuptools import find_packages, setup

package_name = 'robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.xml']),
        ('lib/' + package_name, ['robot/main.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tatiana Berlenko, Ilia Nechaev, Gage Lankford',
    maintainer_email='tatiana.berlenko@gmail.com, ilia.nechaev@jetbrains.com, lankford@mit.edu',
    description="Receives commands from control room, converts them to the Duckiebot's API and calls the Duckiebot's API",
    license='GPLv3',
)
