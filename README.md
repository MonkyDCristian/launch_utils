# launch_utils
ROS2 pkg with utils functions to use [ros2 launch pkgs](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html)

## Description
The functions of this pkg is to avoid duplicate code and long code syntax that comes with ROS2 launch system and setup.py file.
To use the launch_utils pkg you just have to use this name folders structure in your ROS2 packages:
```
config: folder for yours .yaml files configurations.
launch: Folder for yours launch files writed in .launch.py format file.
<pkg-name>: Folder for yours python nodes.
urdf: Folder for yours xacro, URDF and 3D mesh components files.
rviz: Folder for yours rviz configs files.
```
This format can be changed by parameters to meet your needs, check the source code in the scripts in launch_utils folder and the examples to learn how to use it.

## Dependencies
* Framework: [ROS2 Humble (desktop)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

## Install and Compile
```
cd <your_ros2_ws>/src/
git clone https://github.com/MonkyDCristian/launch_utils.git
cd ..
colcon build --packages-select launch_utils
```
Install dependencies 
```
cd <your_ros2_ws>
rosdep install -i --from-path src --rosdistro humble -y
```

## Documentation
The utils functions are contained in [launch_utils/utils.py](https://github.com/MonkyDCristian/launch_utils/blob/main/launch_utils/utils.py), check the files [setup.py](https://github.com/MonkyDCristian/launch_utils/blob/main/setup.py), [example.launch.py](https://github.com/MonkyDCristian/launch_utils/blob/main/launch/example.launch.py) and [example2.launch.py](https://github.com/MonkyDCristian/launch_utils/blob/main/launch/example2.launch.py) to learn how to use it.

for your setup.py:
```
from launch_utils.utils import add_data_files, add_entry_points
```

for yours launch files .launch.py:
```
from launch_utils.utils import include_launch, get_path

from launch_utils.utils import launch_rviz_node
from launch_utils.utils import launch_robot_state_publisher_node
from launch_utils.utils import launch_joint_state_publisher_node
from launch_utils.utils import launch_static_tf_node
```

### Demo:

Launch example:  include_launch and get_path
```
ros2 launch launch_utils example.launch.py
```

Launch example2: robot_state_publisher and joint_state_publisher
```
ros2 launch launch_utils example2.launch.py
```

Launch example3: launch_static_tf_node
```
ros2 launch launch_utils example3.launch.py
```

## References 
* [ROS2 launch for large projects](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)
* [Xacro to clean up a URDF file](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html)
