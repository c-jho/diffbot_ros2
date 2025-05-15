# diffbot_ros2
ROS 2 Humble packages for a two differential wheel robot using two MDrobot MD100 motor drivers.

You can find instructions for installing the serial library at the following GitHub repository.
[https://github.com/c-jho/MD_controller]

## run
```
# run robot
~$ ros2 run diffbot_core diffbot_core

# run robot description
~$ ros2 launch diffbot_description diffbot_urdf.launch.py

# control robot
~$ ros2 run diffbot_teleop teleop_key_node
