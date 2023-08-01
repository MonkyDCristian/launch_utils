from launch import LaunchDescription
from launch_ros.actions import Node

from launch_utils.utils import include_launch, get_path
from launch_utils.utils import launch_rviz_node

def generate_launch_description():
    turtle_tf2_demo_node = include_launch(
                                package_name='turtle_tf2_py',
                                launch_file= 'turtle_tf2_demo.launch.py'
                            )

    turtlesim2_cfg = get_path(
                        package_name='launch_utils', 
                        file='config.yaml',
                        folder='config'
                    )
    
    turtlesim2_node = Node(
                        package='turtlesim',
                        executable='turtlesim_node',
                        namespace='turtlesim2',
                        name='sim',
                        parameters=[turtlesim2_cfg]
                    )
    
    rviz_node = launch_rviz_node(
                    package_name='turtle_tf2_py',  
                    config_file='turtle_rviz.rviz'
                )
    
    return LaunchDescription([
            turtle_tf2_demo_node,
            turtlesim2_node,
            rviz_node,
        ])
