rsg_urdfloader
==============

ROS package for loading URDF files to the Robot Scene Graph.

Expected to be a shared robot model

Prequisites :

1. Install assimp from source :
          
              git clone https://github.com/assimp/assimp
              cd assimp && mkdir build
              cd build && cmake .. && make



Procedure :

1. Install brics_3d in your system.
2. Place rsg_urdfloader in your catkin workspace.
3. make the catkin workspace.


Usage :

rosrun rsg_urdfloader rsg_urdfloader_node urdffile 
