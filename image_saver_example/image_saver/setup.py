from setuptools import find_packages, setup

package_name = 'imager_saver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['image_saver/image_saver.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ilia Nechaev, Gage Lankford, Artemis Pados',
    maintainer_email='ilia.nechaev@jetbrains.com, lankford@mit.edu, apados@mit.edu',
    description="Package which saves images from Duckiebot's camera to folder on local machine",
    license='GPLv3',
)
