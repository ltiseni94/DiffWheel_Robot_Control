Fundamental Packages for Covid Light Robot Control

add the following lines to your .bashrc file (hidden file in home directory)

export ROS_MASTER_URI=http://192.168.64.100:11311
export ROS_IP=192.168.64.100

set the IP Address of your wifi board:

IP: 		192.168.64.100
Sub: 		255.255.192.0
Gateway:	192.168.64.1

set the IP Address of your ethernet board:

IP: 		192.168.0.100
Sub: 		255.255.192.0
Gateway:	192.168.0.1

execute the two launch file:

roslaunch state_machine sm_startup.launch
roslaunch jetson_csi_cam jetson_csi_cam.launch

Finally run the code on your remote pc to teleoperate
