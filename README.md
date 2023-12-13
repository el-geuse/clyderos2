# clyderos2
## EM501 Group C Project

Using a Petoi Bittle for the implementation, with a Raspberry Pi Zero 2W relaying information to an external PC, both running ROS 2. [Credit](https://www.allisonthackston.com/articles/vscode_docker_ros2.html) for the template.

See pi container:
https://github.com/el-geuse/clyderos2_pi

## Folder Breakdown
Packages broken down in a pretty standard way:

- `clyde_bringup` : Contains launch files used in all the other packages
- `clyde_control` : Package for body motion planning and gait generation
- `clyde_description` : Files needed for representation of Clyde's model
- `clyde_driver` : Code needed to communicate with Bittle's serial interface
- `clyde_msgs` : Package detailing all messages defined within the system
- `clyde_nav` : Code used for autonomous navigation
- `clyde_teleop` : Code for basic control using a controller
