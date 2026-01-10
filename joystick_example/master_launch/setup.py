from setuptools import find_packages, setup

package_name = 'master_launch'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ilia Nechaev',
    maintainer_email='ilia.nechaev@jetbrains.com',
    description='Master launch of the joystick',
    license='MIT',
)
