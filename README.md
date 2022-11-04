# Mobile Robot SLAM #

## Remote connection ##

```bash:
export ROS_MASTER_URI=http//192.168.4.10:11311/
```

```bash:
export ROS_IP=<computer ip>
```

## Build workspace ##

```bash:
cd /catkin_ws
catkin_make
```

## Enable environment ##

```bash:
source devel/setup.bash
```

## ROS commands ##

### Enable RGBD camera ###

```bash:
roslaunch realsense_camera r200_nodelet_rgbd.launch
```

### See image through image viewer ###

#### RGB ####

```bash:
rosrun image_view image_view image:=/camera/rgb/image_raw
```

#### Depth ####

```bash:
rosrun image_view image_view image:=/camera/depth/image_rect_raw
```

### Enable serial communication to hardware interface ###

```bash:
rosrun rosserial_python serial_node.py
```

### Enable teleop_twist_keyboard ###

```bash:
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
