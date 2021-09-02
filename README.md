Packages Version: v3.4.0

# Introduction
This package can send control command to real robot from ROS. You can do low-level control(namely control all joints on robot) and high-level control(namely control the walking direction and speed of robot).

This version is suitable for unitree_legged_sdk v3.4, namely Go1 robot. 
As for Aliengo or A1, please use the v3.2 release version of this package and unitree_legged_sdk v3.2.

## Packages:

Basic message function: `unitree_legged_msgs`

The interface between ROS and real robot: `unitree_legged_real`

# Dependencies
* [unitree_legged_sdk](https://github.com/unitreerobotics): v3.4

# Configuration
Before compiling this package, users have to modify the path to unitree_legged_sdk under the CMakeLists.txt of unitree_legged_real.
Just search the "/home/bian/Robot_SDK/unitree_legged_sdk" and change it to your own path. If you are going to compile this package on a computer which is not AMD64 platform, then you have to search "libunitree_legged_sdk.so" under the CMakeLists.txt and change it to the .so file name which is suitable to your computer.

# Build
You can use catkin_make to build ROS packages. First copy the package folder to `~/catkin_ws/src`, then:
```
cd ~/catkin_ws
catkin_make
```

# Setup the net connection
First, please connect the network cable between your PC and robot. Then run `ifconfig` in a terminal, you will find your port name. For example, `enx000ec6612921`.

Then, open the `ipconfig.sh` file under the folder `unitree_legged_real`, modify the port name to your own. And run the following commands:
```
sudo chmod +x ipconfig.sh
sudo ./ipconfig.sh
```
If you run the `ifconfig` again, you will find that port has `inet` and `netmask` now.
In order to set your port automatically, you can modify `interfaces`:
```
sudo gedit /etc/network/interfaces
```
And add the following 4 lines at the end:
```
auto enx000ec6612921
iface enx000ec6612921 inet static
address 192.168.123.162
netmask 255.255.255.0
```
Where the port name have to be changed to your own.

# Run the package
You can control your real Go1 robot from ROS by this package.

```
roslaunch unitree_legged_real real.launch ctrl_level:=highlevel
```
This command will launch a LCM server. The `ctrl_level` means the control level, which can be `lowlevel` or `highlevel`(case does not matter), and the default value is `highlevel`. Under the low level, you can control the joints directly. And under the high level, you can control the robot to move or change its pose.

In order to send message to robot, you need to run the controller in another terminal:
```
rosrun unitree_legged_real position_lcm
```
We offered some examples. When you run the low level controller, please make sure the robot is hanging up. The low level contains:
```
position_lcm
velocity_lcm
torque_lcm
```

And when you run the high level controller, please make sure the robot is standing on the ground. The high level exmaple only has `walk_lcm`.