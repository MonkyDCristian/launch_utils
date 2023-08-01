from launch import LaunchDescription

from launch_utils.utils import launch_rviz_node
from launch_utils.utils import launch_robot_state_publisher_node
from launch_utils.utils import launch_joint_state_publisher_node

def generate_launch_description():
    
    rsp_node = launch_robot_state_publisher_node(
                                        package_name='launch_utils',
                                        xacro_file='wamv_base.urdf.xacro'
                                        )
    
    jsp_node = launch_joint_state_publisher_node(gui=False)
    
    rviz_node = launch_rviz_node(
                    package_name='launch_utils',  
                    config_file='wam_v.rviz'
                )

    return LaunchDescription([
            rsp_node,
            jsp_node,
            rviz_node,
        ])
