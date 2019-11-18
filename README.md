# ArmStrong - Niryo One ROS stack

This is a fork of the [official Niryo One ROS stack](https://github.com/NiryoRobotics/niryo_one_ros). 
Please refer the Niryo One documentation for installation and setup of the environment.

The aim of the ArmStrong project is to turn Niryo One to a real assistive robot for people with upper limb immobility like muscular dystrophy.
In this way, we work to improve robot control through wheelchair joystick but also explore different kinds of user interfaces.

**This is a WIP project so it could be unstable !**

## Features (what's differ from the original Niryo One ROS stack ?)

The main contribution are the qpoases_ros and orthopus_space_control ROS package.
Here is an overview of the features : 
* space control ability 
* multiple user interface (Spacenav, XBox, WebApppp) and a common interface to add new HMI.
* video feedback (WebApp only)

### orthopus\_space\_control package

TODO : Add global overview of the control scheme 

### qpoases_ros package

qpOASES is a quadratic programming solver.
Please refer to the README.md in the qpoases_ros package of more information.

## Installation
### How to install Niryo One ROS packages on your computer (x86) - Simulation Mode

Requirements :
* Ubuntu 16.04
* ROS kinetic  (other versions are not supported)

First install ROS kinetic "Desktop-Full" (tutorial [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)).

You'll need to install some additional ROS packages :
```
sudo apt-get install ros-kinetic-robot-state-publisher ros-kinetic-moveit ros-kinetic-rosbridge-suite ros-kinetic-joy ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-tf2-web-republisher
```
You'll also need to install an additional Python module :
```
sudo -H pip install jsonpickle
```
Create a catkin workspace and clone Niryo One ROS stack :
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:ArmStrong-Robotics/niryo_one_ros.git .
```
Install dependencies :
```
cd ~/catkin_ws
rosdep update
rosdep install qpoases_ros -y
rosdep install orthopus_space_control --ignore-src -y
```
Build the packages :
```
catkin_make
```

**Don't forget to use those commands before you try to launch anything (you can add them in your .bashrc) :**
```
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

Launch the complete Niryo One ROS Stack :
```
roslaunch niryo_one_bringup desktop_rviz_simulation.launch
```

The main differences between this launch file and the launch file executed on Raspberry Pi 3B (rpi\_setup.launch) is that the hardware functionalities are disabled, and you get a 3D simulation view with Rviz.

Note that Niryo One ROS packages have been developed with **ROS kinetic, on Ubuntu 16.04**. Other ROS versions and OS distributions are not supported.

### How to install the web interface (WebApp) ?

The main interface to control the robot is a simple web page. 
That is why you will need a web server like **NGINX** to host your web page. 
Install **NGINX** :
```
sudo apt-get install nginx
```
Open the file '''/etc/nginx/sites-enabled/default''' to configure the **NGINX** and find the line :
```
root /var/www/html;
```
Replace it with (use your username):
```
root /home/<your_username>/catkin_ws/src/orthopus_space_control/scripts;
```

## Usage
### How to use WebApp
### How to use Spacenav
#### Limitation
### How to use XBox Controller
#### Limitation



