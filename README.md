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
