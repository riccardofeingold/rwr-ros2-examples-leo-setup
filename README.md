# rwr-ros2-examples
Examples for the course [real world robotics](https://rwr.ethz.ch/)


## Franka Emika Panda setup @ SRL

This example package explains how to use SRL's Franka Emika Panda Robot. 

This repo focuses on the communication via ROS2 Humble.

### Control system overview

The robot is controlled using an Intel NUC running a realtime kernel. The robot server provides FCI (Franka Control Interface), which provides the interface to be controlled programmatically via [libfranka](https://frankaemika.github.io/docs/libfranka.html). 

The NUC is to be used for low-level nodes that directly interact with the robot. 
Installed in NUC are the ROS1 system that hosts the franka low level controllers and a ROS1 bridge. The low level controllers listen to twist or pose commands and control the robot at a very high frequency for accurate and smooth behaviors.

The ROS1 bridge bridges the franka controller to the ROS2 system, so that all the device in the same local area network (LAN) can send ROS2 command to the controller. 
All planning, perception and other high-level stuff should run on your computer and sent over the network.

The following subsections are the steps of controlling and interfacing with frnaka. The first two steps (power on robot and launch NUC controller) are normally handled by TA. We encourage interested students have a look what's going on.

### Power on the robot and activate FCI

After the robot is powered on (yellow light), connect your computer to the switch, and make sure that you are in the same LAN with the franka and NUC. The IP of our franka is always within the subnet 172.16.0.0 ~ 172.16.0.255, the exact ip of the franka is writen on its marker. Make sure that your ip is also 172.16.0.xxx, you can run `ifconfig` to check your ip.

With the computer joined in the LAN, you can access to the web interface of the franka arm by opening up a browser and goto the ip address of franka `https://172.16.0.xx`. In the web interface you can lock/unlock the brakes, check its status and activate FCI mode.

To activate FCI mode, open Desk, then release the robot brakes, and expand the menu in the top bar. Finally, click on ‘Activate FCI’.

White light means the protective stopc is not released (white light), release it so that you can control the robot through FCI (blue light)

### Launch controllers on NUC

SSH into NUC, and find two setup scripts in `~/rc`. `~/rc/ros1` is for sourcing the ROS1 system, where as `~/rc/ros_bridge` is for starting the bridge.

> Note: these rc scripts should be sourced whenever a new terminal is started. Because we need to choose between different systems to load for each terminal, so we seperate them into individual rc files instead of putting them into `.bashrc`

In a terminal start ros1 and launch controller
```bash
source ~/rc/ros1
roslaunch franka_controllers vel_impedance.launch
# or launch the following, depending which command you want to send
roslaunch franka_controllers pos_impedance.launch
```

In another terminal start rosbridge
```bash
source rc/ros_bridge
ros2 run ros1_bridge dynamic_bridge
```

### Compile and run example

We assume you have ros2 humble installed. A very simple solution based on conda is [robostack](https://robostack.github.io/)

In your ros2 workspace e.g.(`~/ros2_ws`), create `~/ros2_ws/src`, clone the repo.

```
git clone https://github.com/srl-ethz/rwr-ros2-examples
```

Compile it with `colcon`, which is similar to `catkin` in ROS.

```
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

We provide two examples:

**Joy stick**: which is a c++ node that read joy stick and sends twist command
```
ros2 launch rwr-ros2-examples joy_launch.yaml
```

Make sure you have joystick plugged in. on the joystick, press and hold START button to 3 seconds to start sending commands. The interface is quite intuitive, so you will be able to figure it out in a a minute by playing with the joystick. To exit, press the BACK button on the joystick.

NOTE: The joystick has to be in "D" mode (there is a switch at the bottom) and the mode LED should not be ON (press to toggle).

**Pose cmd publisher**: which is a python script to publish a sequence of pose command

```
ros2 launch rwr-ros2-examples franka_pose_cmd_launch.yaml
```