from setuptools import find_packages, setup
from launch_utils.utils import add_data_files

package_name = 'launch_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        add_data_files(package_name, folder_name='launch', file_types=".launch.py"),
	    add_data_files(package_name, folder_name='config', file_types=".yaml"),
	    add_data_files(package_name, folder_name='rviz', file_types=".rviz"),
        add_data_files(package_name, folder_name='urdf', file_types=".urdf.xacro")
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='novi',
    maintainer_email='cristian.nova@uc.cl',
    description='ROS2 pkg with utils functions to use launch pkgs',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = launch_utils.my_node:main'
        ],
    },
)
