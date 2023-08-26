from setuptools import find_packages, setup
from launch_utils.utils import add_data_files, add_entry_points

package_name = 'launch_utils'

data_files = [('share/ament_index/resource_index/packages', ['resource/' + package_name]),
              ('share/' + package_name, ['package.xml'])]

# directory of folder name (key) with script type (value) that will be copy to share folder
folder_file_dir = {'launch':'.launch.py', 
                   'config':'.yaml', 
                   'rviz':'.rviz', 
                   'urdf':'all'}

data_files = add_data_files(data_files, package_name, folder_file_dir)

entry_points = add_entry_points(['my_node'], package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='novi',
    maintainer_email='cristian.nova@uc.cl',
    description='ROS2 pkg with utils functions to use launch pkgs',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points=entry_points,
)
