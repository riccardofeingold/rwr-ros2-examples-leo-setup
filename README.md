# rwr-ros2-examples
Examples for the course [real world robotics](https://rwr.ethz.ch/)


## Franka Emika Panda setup @ SRL

This example package explains how to use SRL's Franka Emika Panda Robot. 

This repo focuses on the communication via ROS2 Humble.

### Control system explaination

The robot is controlled using an Intel NUC running a realtime kernel. The robot server provides FCI (Franka Control Interface), which provides the interface to be controlled programmatically via [libfranka](https://frankaemika.github.io/docs/libfranka.html). 
A detailed documention is located under https://frankaemika.github.io/docs/

The NUC is to be used for low-level nodes that directly interact with the robot. All planning, perception and other high-level stuff should run on your computer and sent over the network.

Installed in NUC are the ROS1 system that hosts the franka low level controllers. The low level controllers listen to twist or pose commands and control the robot at a very high frequency for accurate and smooth behaviors.

### Power on the robot and activate FCI

### Launch controllers on NUC

Create your catkin workspace, clone all packages (including this example package), and build it:
```
TODO
```

Power on the robot's control box and open the Panda Desk Enviroment in a browser:
```
https://172.16.0.2
```

Unlock the brakes, make sure the end-effector settings are right, and finally enable FCI.

Now run the trajectory executation example of this package:
```
roslaunch panda_srl trajectory.launch
```

### Compile and run examples

This executes an example joint space trajectory. The trajectory is defined in a yaml file within the folder trajectories. 
The yaml file can be changed to plan in task space, adapt cycles and waiting times, and change the trajectory points itself.
If you want to use another trajectory file, run:
```
roslaunch panda_srl trajectory.launch trajectory:=path_to_trajectory.yaml
```

Another mode of controlling this robot is realtime tele-op using a joystick. This requires a joystick connected to the NUC and twist commands published by the joystick node. To run this, run:
```
roslaunch panda_srl teleop.launch
```

Press ENTER key to start. At this point the robot will move to a predefined initial configuration. Then, on the joystick, press and hold START button to 3 seconds to start sending commands. The interface is quite intuitive, so you will be able to figure it out in a a minute by playing with the joystick. To exit, press the BACK button on the joystick.

NOTE: The joystick has to be in "D" mode (there is a switch at the bottom) and the mode LED should not be ON (press to toggle).
