# https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html
# https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html

import os
from glob import glob

from ament_index_python.packages import get_package_share_directory 

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command

from launch_ros.actions import Node as LaunchNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def add_data_files(data_files=[], package_name='pkg', folder_file_dir={'launch':'.launch.py'}):
    """
        folder_name / file_types
        launch      / .launch.py
        config      / .yaml
        rviz        / .rviz
        urdf        / .xacro (or all if you have models inside)
    """
    for folder_name, files_type in folder_file_dir.items():
        if files_type != "all":
            folder_path = os.path.join('share', package_name, folder_name)
            file_names = glob(os.path.join(folder_name, f'*{files_type}'))
            data_files.append((folder_path, file_names))
        
        else:
            for (path, directories, file_names) in os.walk(folder_name):
                if len(file_names) > 0:
                    folder_path = os.path.join('share', package_name, path)
                    filenames = glob(os.path.join(path, f'*.*'))
                    data_files.append((folder_path, filenames))
    
    return data_files

def include_launch(package_name="pkg", launch_file="example.launch.py", launch_folder='launch', launch_arguments=None):
    return IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare(package_name),
                        launch_folder,
                        launch_file
                    ])
                ]),
                launch_arguments=launch_arguments
            )

def get_path(package_name='pkg', file='config.yaml', folder='config'):
    return os.path.join(
                get_package_share_directory(package_name), 
                folder, 
                file
            )

def launch_rviz_node(package_name="pkg", config_file="cfg.rviz", rviz_folder='rviz'):
    rviz_config = get_path(package_name, config_file, rviz_folder)
    
    rviz_node = LaunchNode(
                    package='rviz2', 
                    executable='rviz2', 
                    name='rviz2', 
                    arguments=['-d', rviz_config]
                )
    
    return rviz_node

def launch_robot_state_publisher_node(package_name='pkg', xacro_file='robot.urdf.xacro', urdf_folder='urdf'):
    path_to_urdf = get_path(package_name, xacro_file, urdf_folder)
    
    robot_description = ParameterValue(Command(['xacro ', str(path_to_urdf)]), value_type=str)
    
    rsp_node = LaunchNode(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    parameters=[{
                        'robot_description': robot_description
                    }]
                )
    
    return rsp_node

def launch_joint_state_publisher_node(gui=False):
    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher = "joint_state_publisher"
    
    if gui:
        joint_state_publisher = "joint_state_publisher_gui"

    jsp_node = LaunchNode(
                    package=joint_state_publisher,
                    executable=joint_state_publisher
                )
    return jsp_node

    
