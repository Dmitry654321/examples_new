from setuptools import find_packages, setup

package_name = 'tof_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/'+package_name,['tof_reader/tof_node.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gage Lankford, Ilia Nechaev',
    maintainer_email='lankford@mit.edu, ilia.nechaev@jetbrains.com',
    description='Move Duckiebot forward until it detects an object within 0.2 meters',
    license='GPLv3',
)
