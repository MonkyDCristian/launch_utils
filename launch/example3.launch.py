from launch import LaunchDescription

from launch_utils.utils import launch_rviz_node
from launch_utils.utils import launch_static_tf_node

def generate_launch_description():
    
    base_link = launch_static_tf_node(parent_frame='world', child_frame='base_link', translation = [0, 0, 1])
    base_link2 = launch_static_tf_node(parent_frame='base_link', child_frame='base_link2', translation = [-0.5, 0, 0], rotation=[0, 0, 1.57])
    rviz_node = launch_rviz_node(config_file='default')
    rviz_node = launch_rviz_node(package_name='launch_utils', config_file='static_tf.rviz')
   
    return LaunchDescription([
            base_link,
            base_link2,
            rviz_node,
        ])
