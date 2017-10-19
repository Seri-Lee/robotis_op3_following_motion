# ROBOTIS-OP3 motion following demo using openpose_ros

## 1. Overview  
The motion following demo uses openpose which is available as a ROS package.  
 | Reference : [openpose](https://github.com/CMU-Perceptual-Computing-Lab/openpose)  
 | Reference : [openpose_ros](https://github.com/ildoonet/ros-openpose)  
As the openpose is based on machine learning algorithm, additional PC with nVidia GPU will be required in order to process the algorithm for ROBOTIS-OP3.  
Please refer to below configuration for the motion following demo. 
![System Structure](https://github.com/Seri-Lee/robotis_op3_following_motion/blob/master/doc/system_structure.png?raw=true)

This demo will manipulate arm roll and elbow joint in order to imitate the skeleton recognized via camera.

## 2. How to set and install
1. How to set  
 - network  
    Connect ROBOTIS-OP3 wifi hotspot from an external PC  
    Configure ROS network in the external PC(ROS_MASTER_URI=http://10.42.0.1:11411)  

 - timing  
     | Reference : [ROS Timing issue](http://wiki.ros.org/ROS/NetworkSetup#Timing_issues.2C_TF_complaining_about_extrapolation_into_the_future.3F)

2. How to install  
 - ROBOTIS-OP3  
     Update to the latest source code for ROBOTIS-GIT/ROBOTIS-OP3(direct_control_module is required)  

 - external PC  
     1. [Install openpose](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation.md)  

     2. [Install openpose_ros](https://github.com/ildoonet/ros-openpose#installation)  

     3. Install robotis_op3_following_motion  
       ```
       $ cd ~/catkin_ws/src
       $ git clone https://github.com/Seri-Lee/robotis_op3_following_motion.git
       $ cd ~/catkin_ws
       $ catkin_make
       ```


## 3. How to run
1. Run op3_manager in ROBOTIS-OP3
```
$ roslaunch op3_manager op3_manager.launch
```  

2. Run usb_cam_node in ROBOTIS-OP3
```
$ roslaunch ball_detector ball_detector_from_usb_cam.launch
```  

3. Run openpose_ros_node and robotis_op3_following_motion_node in external PC
```
$ roslaunch robotis_op3_following_motion openpose_op3.launch
```  


## 4. How to operate
Control with buttons on the back  

- mode button : Init pose for demo and start the Motion Following demo  
- start button : Pause demo / Resume Demo  
- user button : Stop Demo and return to ROBOTIS-OP3 init pose.  
