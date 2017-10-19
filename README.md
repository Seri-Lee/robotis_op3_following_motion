# ROBOTIS-OP3 motion following demo using openpose_ros

## 1. Overview
openpose라는 매혹적인 프로그램이 눈에 들어옴  
이를 이용한 데모를 만들고 싶어 찾아보니 ROS용으로 만들어진 패키지가 있음  
 | Reference : [openpose](https://github.com/CMU-Perceptual-Computing-Lab/openpose)  
 | Reference : [openpose_ros](https://github.com/ildoonet/ros-openpose)  
하지만 머신러닝을 기반으로 하고 있어 ROBOTIS-OP3 단독으로는 하기 힘듬, nvidia GPU가 있는 PC가 추가적으로 필요함(openpose 실행용)    
아래와 같은 구성으로 데모를 구성해 봄  

## 2. How to set and install
1. How to set  
 - network  
    external PC에서 ROBOTIS-OP3의 핫스팟에 무선 연결
    external PC의 ROS network 세팅을 해줌(ROS_MASTER_URI=http://10.42.0.1:11411)

 - timing  
     | Reference : [ROS Timing issue](http://wiki.ros.org/ROS/NetworkSetup#Timing_issues.2C_TF_complaining_about_extrapolation_into_the_future.3F)

2. How to install  
 - ROBOTIS-OP3  
     ROBOTIS-GIT/ROBOTIS-OP3의 최신 소스 적용(direct_control_module 필요)  

 - external PC  
     1. [openpose 설치](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation.md)  

     2. [openpose_ros 설치](https://github.com/ildoonet/ros-openpose#installation)  

     3. robotis_op3_following_motion 설치  


## 3. How to run



## 4. How to operate
