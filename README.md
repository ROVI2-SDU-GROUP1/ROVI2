# Install notes

## Stereo camera(Primesense)
### Install
#### Ubuntu/Debian
```sh
sudo apt-get install ros-kinetic-openni2-launch
```
#### Arch
Ros-Kinetic is not available in the default repositories, but can instead be installed from AUR. Example using pacaur:
```sh
pacaur -S --noconfirm  ros-kinetic-ros-core ros-kinetic-robot ros-kinetic-openni2-launch
```
### Usage:
On Arch, you need to source /opt/ros/kinetic/setup.sh before using ros commands.
```h
roslaunch openni2_launch openni2.launch	
```

# ROS
## Start bumblebee node on PC
```sh
roslaunch pointgrey_camera_driver bumblebee.launch
```

## Connect to remote ROS core
```
export ROS_MASTER_URI=http://192.168.100.53:11311

## Listen on some interface
```
export ROS_IP=192.168.100.10
```
In the case above, the IP address of my laptop is 192.168.100.10.


# Networking
The ip address of the PC in Cell3 is:
192.168.10.53

## Dependencies for UR sim
```sh
sudo apt-get install libjava3d-java libbiojava3-java 
sudo apt-get install libxmlrpc-c++8v5:i386 
```

Then run:
```sh
./ursim-3.X/start-ursim.sh
```
