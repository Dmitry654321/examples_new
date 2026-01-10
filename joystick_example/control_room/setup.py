from setuptools import find_packages, setup

package_name = 'control_room'

setup(
    name=package_name,
    version='1.0.0',
    packages=['utils'],
    package_dir={'': 'include'},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.xml']),
        ('lib/' + package_name, ['control_room/main.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tatiana Berlenko, Ilia Nechaev, Gage Lankford',
    maintainer_email='tatiana.berlenko@gmail.com, ilia.nechaev@jetbrains.com, lankford@mit.edu',
    description='Reads keys from keyboard and publishes them to robot node to later send to Duckiebot',
    license='GPLv3',
)
