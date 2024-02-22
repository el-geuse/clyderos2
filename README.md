# clyderos2
## EM501 Project
### Group C

Master's Thesis group project working on creating a quadruped that can be used in elderly care for fall detection and supervision! Built using ROS 2, with a Raspberry Pi Zero 2W attached to the quadruped frame, issuing data to the master pc.

## Folder Breakdown
Packages broken down in a pretty standard way:

1. **`clyde_bringup`** : Contains launch files used in all the other packages.
2. **`clyde_control`** : Package for body motion planning and gait generation.
3. **`clyde_description`** : Files needed for representation of Clyde's model.
4. **`clyde_driver`** : Code needed to communicate with Bittle's serial interface.
     - Contains the code to be run on the raspberry pi.
     - Node 1 : `\vision`
           - 
     - Node 2 : `\audio`
           -
     - Node 3 : `\movement`
           -
6. **`clyde_msgs`** : Package detailing all messages defined within the system.
7. **`clyde_nav`** : Code used for autonomous navigation.
8. **`clyde_teleop`** : Code for basic control using a controller.
