\mainpage
# caros_camera #
caros_camera is a ROS node for communicating with cameras often used in vision applications. Currently supported cameras are: 
* [Basler Ace (GigE)](http://www.baslerweb.com/en/products/area-scan-cameras/ace?tab=gige&gclid=CIeC0Mf8gMoCFar4cgodOswH2g)
* [Ensenso N20 (GigE) - Stereo 3D Camera](http://www.ensenso.com/portfolio-item/n20/)
* [Point Grey BumbleBee2 (FireWire) - Stereo](https://www.ptgrey.com/bumblebee2-firewire-stereo-vision-camera-systems)

# Interfaces - how to use this node #
This node publishes one or two images using the @ref caros::CameraServiceInterface interface. The raw image of a single camera is available at:

    /camera_name/caros_camera/namespace/image_raw

if the camera is a combined stereo camera like the BumbleBee2 and Ensenso N20, two images will be available at:

    /camera_name/caros_camera/left/image_raw
    /camera_name/caros_camera/right/image_raw

Which can be subscribed to through ROS. Notice that the **camera_name** and the **namespace** can be specified as arguments at launch time (see Launching the node) this is useful for stereo and multiple camera setups. A service interface proxy is also available for the node @ref caros::CameraSIProxy where single images can be retrieved, formatted as [OpenCV Mat](http://docs.opencv.org/2.4/modules/core/doc/basic_structures.html#Mat), and camera information such as the projection matrix can be retrieved as well. 
# Requirements #
The caros_camera node requires OpenCV to be installed, more specifically it requires opencv_bridge to convert between ROS images and OpenCV images.

When compiling the caros_camera node, it will automatically try to detect which cameras your system support.  

* The caros_camera node requires the [libdc1394](http://damien.douxchamps.net/ieee1394/libdc1394/) library to be able 
to communicate with IEEE 1394 (Firewire) based cameras. (eg. PointGrey BumbleBee2). This library is often installed by 
default on common linux distros like Ubuntu. Hence no action required. (Not true for Ubuntu 16.04 atm.)
Furthermore [FlyCapture2](https://www.ptgrey.com/flycapture-sdk) must be installed. Make sure you read the README and 
use the install_flycapture.sh script to install FlyCapture2 appropiately.
* The caros_camera node requires the [PylonSDK 3.2.1](http://www.baslerweb.com/media/documents/pylon-3.2.1-x86_64.tar.gz) from Basler to be installed, in order to support Basler's Ace GigE cameras. Follow install instructions included in pylon-3.2.1-x86_64.tar.gz
* The caros_camera node requires the [EnsensoSDK](http://www.ensenso.com/support/sdk-download/) to be installed, as well as the camera driver [uEye Driver](http://www.ensenso.com/support/sdk-download/). Furthermore [PCL 1.8.0](https://github.com/PointCloudLibrary/pcl/releases) must be installed. If PCL is not installed in its default location, you can use the PCL_DIR environment variable to your build path. 

If none of the above mentioned drivers and libraries are found on your system, the caros_camera node won't build. If you need additional camera support, create an issue stating your requirements.
# Launching the node #
The CAROS Camera node can be launched by using the following:

    roslaunch caros_camera caros_camera.launch

Typically arguments should be specified as well, like the following example (see Parameters):

    roslaunch caros_camera caros_camera.launch camera_name:="basler_left" namespace:="left" basler_serial:="21420534" calibrated:="1"

When using BumbleBee2 stereo camera or the Ensenso N20 it is important to set the stereo parameter to "true", and use the default namespace parameter which is "left":

    roslaunch caros_camera caros_camera.launch camera_name:="bumblebee" stereo:="true" bumblebee_serial:="11072182" calibrated:="1"

Along with the caros_camera node a [image_proc](http://wiki.ros.org/image_proc) node is also started. This node will perform image rectification based on the calibration file which is automatically loaded from the camera serial number if the calibrated launch argument is set to "1" and if the calibration file is available in the calibration_files folder. This means that images which are rectified and with or without color information are available at:

    /camera_name/caros_camera/namespace/image_mono
    /camera_name/caros_camera/namespace/image_rect
    /camera_name/caros_camera/namespace/image_color
    /camera_name/caros_camera/namespace/image_rect_color

If a BumbleBee2 stereo camera is used, all of the above will be available for the 'left' and 'right' camera, as well as a disparity map and a stereo pointcloud produced by the [stereo_image_proc](http://wiki.ros.org/stereo_image_proc) node, available at:

    /camera_name/caros_camera/disparity
    /camera_name/caros_camera/points2

For Ensenso and the BumbleBee2 camera the caros_camera node will launch [rviz](http://wiki.ros.org/rviz) showing the raw 
 and rectified images of the respective camera. The rviz configuration can be modified in the config folder. 
 
 NOTICE! 
 the camera_name argument specified upon launch must match the frame name in the rviz configuration, in order for rviz to show the images. For 
 Ensenso the camera_name argument must be then be 'ensenso', and for BumbleBee2 in must be 'bumblebee'. 

## Parameters ##
The following parameters are supported:


| Parameter | Description | Default |
| --------- | ----------- | ------- |
| camera_name | Can provide a unique identifier for the given camera, when using multiple cameras | camera |
| namespace | Can denote a camera as being the left or right camera for stereo, or a custom sub-namespace | left |
| basler_serial | Serial number of a basler camera | 0 |
| stereo | A bool indicating if a stereo camera is used | 0 (false) |
| bumblebee_serial | Serial of the bumblebee camera | 0 |
| ensenso_serial | Serial of the ensenso camera | 0 |
| calibrated | If '1' the caros_camera node will look for calibration files in the calibration_files folder | 0 |

# Dynamic Reconfigurable Parameters #
The caros_camera node supports dynamic reconfigurable parameters for all currently supported cameras. Which means it is possible to alter the camera parameters during runtime. The parameters can be reconfigured through the [rqt_reconfigure](http://wiki.ros.org/rqt_reconfigure) GUI,
which can be launched using the following:

    rosrun rqt_reconfigure rqt_reconfigure

Please see the cfg folder of the caros_camera node to see which dynamic reconfigurable parameters are available for each
camera.

# Small demo(s) #
No demos yet.
