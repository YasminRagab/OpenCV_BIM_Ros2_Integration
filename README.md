# Glass Crane Operation Assistance

## Description

ROS2 package uses OpenCV aruco marker pose detection and BIM integration for glass crane operation assistance

## Getting Started

Make sure you have [ROS2](https://docs.ros.org/en/humble/Installation.html) Installed on your machine. Create a workspace

```
$ mkdir -p ptp_ws/src/
```
* Make sure you have a ROS2 Driver for V4L USB Cameras.
* If not clone the usb cam demo ros2 branch into the src folder alongside this repository [usb_cam](http://wiki.ros.org/usb_cam)
* Clone this package into your source folder and don't forget to clone the ros2_aruco_interfaces too.

```
$ cd ptp_ws/src
$ git clone https://github.com/ros-drivers/usb_cam
$ git clone https://git.rwth-aachen.de/prototypingproject20222023/ros2_aruco.git
$ git clone https://git.rwth-aachen.de/prototypingproject20222023/ros2_aruco_interfaces.git

```
* Build the package

```
$ cd .. 
$ colcon build 
```
* Source the overlay

```
$ . install/setup.bash
```
## Running the Code
` Option 1 ` 

* Launch the camera

```
$ ros2 launch usb_cam demo_launch.py
```
* Make sure your joystick is connected
* Launch the package

``` 
$ ros2 launch ros2_aruco myrobot_launch.py
```
` Option 2 `

* In the desktop_icon folder, change the path in each file 

    * (Exec=/home/`user`/ptp_ws/src/ros2_aruco/launch_script/pkg_launch.sh)
    * (Icon=/home/`user`/ptp_ws/src/ros2_aruco/launch_script/robot.png)

* Copy the content of the folder to your desktop.

* Run the Package.
