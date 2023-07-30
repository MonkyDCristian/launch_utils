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

def add_data_files(package_name='pkg', folder_name='launch', file_types=".launch.py"):
    """
        folder_name / file_types
        launch      / .launch.py
        config      / .yaml
        rviz        / .rviz
        urdf        / .urdf.xacro
    """
    return (os.path.join('share', package_name, folder_name), glob(os.path.join(folder_name, f'*{file_types}')))

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

def get_cfg(package_name='pkg', config_file='config.yaml', config_folder='config'):
    return os.path.join(
                get_package_share_directory(package_name), 
                config_folder, 
                config_file
            )

def launch_rviz_node(package_name="pkg", config_file="cfg.rviz"):
    rviz_config = os.path.join(
                        get_package_share_directory(package_name),
                        'rviz',
                        config_file
                    )
    rviz_node = LaunchNode(
                    package='rviz2', 
                    executable='rviz2', 
                    name='rviz2', 
                    arguments=['-d', rviz_config]
                )
    return rviz_node

def launch_robot_state_publisher_node(package_name='pkg', xacro_file='robot.urdf.xacro', urdf_folder='urdf'):
    path_to_urdf = os.path.join(
                        get_package_share_directory(package_name), 
                        urdf_folder, 
                        xacro_file
                    )
    
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

    
