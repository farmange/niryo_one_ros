# ArmStrong - Niryo One ROS stack

This is a fork of the [official Niryo One ROS stack](https://github.com/NiryoRobotics/niryo_one_ros). 
Please refer the Niryo One documentation for installation and setup of the environment.

The aim of the ArmStrong project is to turn Niryo One to a real assistive robot for people with upper limb immobility like muscular dystrophy.
In this way, we work to improve robot control through wheelchair joystick but also explore different kinds of user interfaces.

**This is a WIP project so it could be unstable !**

- [ArmStrong - Niryo One ROS stack](#armstrong---niryo-one-ros-stack)
  - [Features](#features)
    - [orthopus\_space\_control package](#orthopusspacecontrol-package)
    - [qpoases_ros package](#qpoasesros-package)
  - [Installation](#installation)
    - [How to install Niryo One ROS packages on your computer (x86) - Simulation Mode ?](#how-to-install-niryo-one-ros-packages-on-your-computer-x86---simulation-mode)
    - [How to install Niryo One ROS packages on your robot ?](#how-to-install-niryo-one-ros-packages-on-your-robot)
    - [How to install the WebApp interface ?](#how-to-install-the-webapp-interface)
  - [Usage](#usage)
    - [How to use WebApp ?](#how-to-use-webapp)
      - [Calibration](#calibration)
      - [Play](#play)
    - [How to use Spacenav ?](#how-to-use-spacenav)
      - [Limitation](#limitation)
    - [How to use XBox Controller ?](#how-to-use-xbox-controller)

## Features

The main contribution are the qpoases_ros and orthopus_space_control ROS package.
Here is an overview of the features : 
* space control ability using brand new IK solving approach base on [Quadratic Programming](https://en.wikipedia.org/wiki/Quadratic_programming)
* multiple user interface ([Spacenav](http://wiki.ros.org/spacenav_node), [XBox](http://wiki.ros.org/joy), [WebApp](#how-to-use-webapp)) and a common interface to add new HMI
* video feedback (WebApp only) using [web_video_server](http://wiki.ros.org/web_video_server) package

### orthopus\_space\_control package

Global overview of the space control scheme :
![space control overview](https://orthopus.com/wp-content/uploads/2019/11/orthopus_space_control.png)

With :

* <img src="https://latex.codecogs.com/svg.latex?\Large&space;q_{current}"/> : the current joint position
* <img src="https://latex.codecogs.com/svg.latex?\Large&space;X_{current}"/> : the current space position
* <img src="https://latex.codecogs.com/svg.latex?\Large&space;\dot{X}_{desired}"/> : the desired space velocity
* <img src="https://latex.codecogs.com/svg.latex?\Large&space;\dot{q}_{command}"/> : the joint velocity command
* <img src="https://latex.codecogs.com/svg.latex?\Large&space;q_{command}"/> : the joint position command (Niryo low level controller type follow_joint_trajectory) to send to the robot
* <img src="https://latex.codecogs.com/svg.latex?\Large&space;q_{measured}"/> : the measured joint position of the robot

**Note: Currently, only open loop mode was tested ($q_{current}$=$q_{command}$). This way, Niryo low level control can be seen as a black box and no perturbation is taken into account.**

### qpoases_ros package

qpOASES is a quadratic programming solver.
Please refer to the README.md in the qpoases_ros package of more information.

## Installation
### How to install Niryo One ROS packages on your computer (x86) - Simulation Mode ?

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
Build the ROS stack :
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

### How to install Niryo One ROS packages on your robot ?

Connect your robot in shh (see Niryo One documentation) and change the github upstream :
```
cd ~/catkin_ws/src
git remote add armstrong git@github.com:ArmStrong-Robotics/niryo_one_ros.git
git fetch armstrong
git pull armstrong master 
```

Install dependencies :
```
cd ~/catkin_ws
rosdep update
rosdep install qpoases_ros -y
rosdep install orthopus_space_control --ignore-src -y
```

Kill the current ROS instance to free RAM space and CPU load :
```
rosnode kill --all
```

Build the ROS stack :
```
catkin_make -j1
```

Launch the complete Niryo One ROS Stack :
```
roslaunch niryo_one_bringup rpi_setup.launch
```

### How to install the WebApp interface ?

**This section is valid for either simulated or real (RPi) robots.**

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
### How to use WebApp ?

Go to http://<IP_OF_THE_ROBOT>/interface.html in your favorite web browser. The web page is responsive so it can also be used on smartphone, tablet, etc. 
Here is an overview of the WebApp :
![WebApp Interface](https://orthopus.com/wp-content/uploads/2019/11/webapp_interface.png)

The status indicator (the spinner above Gripper section) indicate the state of the robot :
* <img src="https://orthopus.com/wp-content/uploads/2019/11/spinner_error.png" width="20" > : connection error, be sure the robot is well started and refresh the page (F5)
* <img src="https://orthopus.com/wp-content/uploads/2019/11/spinner_start.png" width="20" > : connection ok but robot not finish to start 
* <img src="https://orthopus.com/wp-content/uploads/2019/11/spinner_calib.png" width="20" > : connection ok but robot requires calibration ([see calibration section](#calibration))
* <img src="https://orthopus.com/wp-content/uploads/2019/11/spinner_ready.png" width="20" > : connection ok and robot is fully operational. ([see play section](#play))

#### Calibration
If status indicator show a spinning yellow circle, you should probably calibrate the robot (mandatory after a startup). Put the robot in the calibration position as mentionned in Niryo documentation (see arrows on the arm), then click on the ORTHOPUS header of the interface. Robot calibration will be perform and the status indicator should go green.

#### Play
To disable the Niryo One learning mode, simply activate the toogle Enable/Disable. You can now perform a Home position which will then automaticaly enable the space control. 

You can now play with both joysticks (left joystick for up/down/left/right motion and right joystick for up/down/forward/backward motion) and orientation sliders.

**Note : Drink and Lay buttons are used to execute predefined trajectories. This is currently unstable but it will be improved very soon...**

### How to use Spacenav ?
If you want to use spacenav, you have to plug it on Niryo One or simulation PC depending on your target. 
Then edit the following file :
- To use on the Niryo One RPi robot, edit file ~/catkin_ws/src/niryo_one_bringup/config/rpi_ros_processes.yaml and modify lines :
  ```
    - name: 'cartesian_controller'
      launch_on_startup: true
      delay_before_start: 0.0
      cmd: 'roslaunch orthopus_space_control demo.launch'
      args: 
        ['webapp:=true', 'spacenav:=false']
      dependencies:
          - controllers
          - robot_commander
  ```
  with :
  ```
    - name: 'cartesian_controller'
      launch_on_startup: true
      delay_before_start: 0.0
      cmd: 'roslaunch orthopus_space_control demo.launch'
      args: 
        ['webapp:=false', 'spacenav:=true']
      dependencies:
          - controllers
          - robot_commander
  ```
- To use on simulated robot, edit file ~/catkin_ws/src/niryo_one_bringup/launch/desktop_rviz_simulation.launch and modify lines :
  ```
    <include file="$(find orthopus_space_control)/launch/demo.launch">
        <arg name="spacenav" value="false" />
        <arg name="webapp" value="true" />
    </include>
  ```
  with : 
  ```
    <include file="$(find orthopus_space_control)/launch/demo.launch">
        <arg name="spacenav" value="true" />
        <arg name="webapp" value="false" />
    </include>
  ```
Then you can start the robot as mentioned before.

#### Limitation
As the spacenav did not offer as many buttons as the WebApp did, you have to perform calibration, enable/disable and goto home through the WebApp ! Note that joystick of the WebApp will not work when spacenav is activated.

### How to use XBox Controller ?
Should work editing same file as spacenav but I no longer have XBox controller to maintain this feature.



