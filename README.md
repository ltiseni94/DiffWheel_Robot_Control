Fundamental Packages for Covid Light Robot Control

- Install ROS melodic
    
    
    1) setup your sources.list:
    
			$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
            
                	
    2) setup your keys:
    
			$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    
    
    3) Installation:
    
			$ sudo apt update
			$ sudo apt install ros-melodic-desktop
    
    
	4) Environment Setup:
       
			$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    
    
    5) Dependencies:
    
			$ sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
    
    
    6) Initialize rosdep:
    
			$ sudo apt install python-rosdep
			$ sudo rosdep init
			$ rosdep update



- Create a Catkin workspace (to do section)


- Optional: Install Opencv4, realsense, and RTABMAP: follow andrea boscolo camiletto guide.


- Other packages that you need to install:

    1) gscam:

			$ cd catkin_ws/src
			$ git clone https://github.com/ros-drivers/gscam

	modify the file ./gscam/Makefile with this string:
	
			EXTRA_CMAKE_FLAGS = -DUSE_ROSBUILD:BOOL=1 -DGSTREAMER_VERSION_1_x=On

    
    2) image_common:
    
			$ cd catkin_ws/src
			$ git clone https://github.com/ros-perception/image_common
    
    
    3) ydlidar_ros-master:
	
			$ cd catkin_ws/src
			$ git clone -b master --single-branch https://github.com/YDLIDAR/ydlidar_ros
			$ cd
			$ sudo mkdir /dev/ydlidar
			$ cd catkin_ws/src/ydlidar_ros/startup
			$ sudo chmod 777 ./*
			$ sudo sh initenv.sh
    
	in case of errors ignore them.
	
	
    4) joy:
    
			$ sudo apt-get update
    		$ sudo apt-get install ros-melodic-joy
    		$ sudo apt-get install ros-melodic-joystick-drivers



- Configuration:

    1) Add the following lines at the end of .bashrc file (hidden file in home directory):

			export ROS_MASTER_URI=http://ipaddress:port
			export ROS_IP=ipaddress
    
	where ipaddress is the wifi board ip address (example: 192.168.1.100) and the port is the one used by ROS for communication (standard port: 11311)
    
    
    2) set the manual IP Address of your wifi board accordingly (edit connections --> IPv4)
    
    
    3) set the manual IP Address of your ethernet board:

		- IP: 		192.168.0.100
    
		- Sub: 		255.255.192.0
    
		- Gateway:	192.168.0.1



- Build workspace:

        $ rosdep install --from-paths ~/catkin_ws --ignore-src --rosdistro=ROSmelodic
        $ catkin_make
        $ source ~/.bashrc
        $ source devel/setup.bash



- Usage:

execute the launch file:

        $ roslaunch state_machine sm_startup.launch

Finally run the code on your remote pc to teleoperate (use the DiffWheel_Robot_Teleop package).
