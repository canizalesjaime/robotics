# Project Description 
* A compact 3D-printed mobile robot with a 4-DOF servo arm and dual TT motors for navigation.
Integrated LiDAR for obstacle avoidance, a camera with YOLOv8n for real-time object detection, and an MPU6050 IMU for pose estimation.  
* A web-based interface for remote control of the robot and arm, featuring
live video with detection overlays and accelerometer data. Implemented using 
react and tailwind css to design the frontend and fast api to interact with
the backend.
* <img src="./robot-dashboard/public/plexi_bot.JPG" width ="200" >
* [video](./robot-dashboard/public/arm_plexi_bot.mp4)

# System Design

## Software architecture
Main creates an object for each peripheral device which has a class defined: motor wheels, arm, accelerometer, camera, and lidar. Depending on feedback from frontend or physical sensors, call relevant method to determine output. One dockerfile sets up frontend for use on your computer, and
another dockerfile sets up dependencies on raspberry pi 5 to run backend.<br>
<img src="./robot-dashboard/public/software_architecture.png" width ="500" >


## Hardware
* stl files used for 3D printing robot base included in [freecad_files](./free_cad_files)
* pcb board power by a 7.4V LiPo battery 2200mAh, buck converter to 5V to power raspberry pi 5, 7.4V used to power all the motors(4 servo, 2 tt motors)
* The robotic arm shown in was 3D printed using a model created by [FABRI_CREATOR](https://cults3d.com/en/users/FABRI_CREATOR).  
STL file available here: [Mini Robotic Arm on Cults3D](https://cults3d.com/en/3d-model/gadget/mini-robotic-arm)  
Licensed under Cults PU (Personal Use) – No commercial use or AI applications.


### Peripheral Sensors, Motor Drivers Used on Raspberry pi 5
#### Ultrasonic HCSR04(no longer used)
* VCC -> PIN 2(5V)
* TRIG -> PIN 29 (GPIO 5)
* ECHO -> PIN 31 (GPIO 6) with voltage divider
* GND -> PIN 39 (GND)


#### Accelerometer MPU6050 
* VCC -> PCB
* GND -> PIN 6 (GND)
* SCL -> PCB
* SDA -> PCB


#### TB6612 Motor Driver 1
* GND -> LIPO Battery and PIN 9 (GND)
* +12V -> LIPO Battery
* ENA -> PIN 32 (GPIO 12)
* IN1 -> PIN 11 (GPIO 17)
* IN2 -> PIN 13 (GPIO 27)
* IN3 -> PIN 16 (GPIO 23)
* IN4 -> PIN 18 (GPIO 24)
* ENB -> PIN 33 (GPIO 13)
* STBY -> PIN 22 (GPIO 25) 
* VCC -> PIN 17

#### PCA9865
* VCC(3.3) -> PCB
* SDA -> PCB
* SCL -> PCB
* GND -> PIN 34

#### Lidar
* VCC -> PIN 4(5V)
* RX -> PIN 8 (GPIO 14)
* TX -> PIN 10(GPIO 15)
* GND -> PIN 14



# Working Setup
## Web App(Frontend)
* Note: Recommended to use another computer for this part to take some of the load off raspberry pi 5. Still works if you decide to use the raspberry pi 5 just a bit slower.
* works fine on ubuntu with docker setup below or on windows use devcontainers:
1. ```docker build -f ./robot_dashboard.Dockerfile -t web-setup . ```
2. ```docker run -it --rm --net=host --name web-container -v /home/jaime/homebot:/workspace web-setup:latest```

## Backend  
* run on raspeberry pi 5(in container created by dockerfile [humble_pi.Dockerfile](./.devcontainer/humble_pi.Dockerfile)): 
```
uvicorn main:app --host 0.0.0.0 --port 8000
```


# ros2 setup (work in progress)
## workspace setup
1. cd /workspaces/homebot/ros2_ws && colcon build or or colcon build --symlink-install && source install/setup.bash 
2. ros2 run robot_control \<executable_name\>
3. ros2 launch robot_control \<executable_name\>
4. ros2 service call /capture_image std_srvs/srv/Trigger
5. xhost +local:root and xhost -local:root for rviz2 on linux
6. ros2 run tf2_tools view_frames (saves pdf frames)
7. ros2 pkg create my_robot_description --build-type ament_python  (create package)
8. ros2 run rqt_graph rqt_graph (sub-pub architecture)


## docker container to run on pi which is ubuntu based with ros2 installed + peripheral libraries
1. docker build -f ./humble_pi.Dockerfile -t ros2-setup .
2. docker run -it --rm \
  --init \
  --privileged \
  --net=host \
  --device=/dev/gpiomem \
  --device=/dev/mem \
  --device=/dev/ttyAMA0 \
  --cap-add=SYS_RAWIO \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /sys/class/gpio:/sys/class/gpio:cached \
  -v /dev:/dev \
  -v /run:/run \
  --name ros2-container \
  -v /home/jaime/homebot:/workspace \
  ros2-setup:latest \
  /bin/bash

3. simple:
docker run -it --rm --init --privileged --net=host --name ros2-container -v /home/jaime/homebot:/workspaces/homebot jaimec21/jazzy_pi:latest

4. open another terminal for container:
docker exec -it ros2-container bash



# To Do
1. homebot: 
  - lidar, stop base is lagged
  - make rotate_base and stop_base into a service instead(maybe other "actions" as well)
  - slam/urdf file for frames and kinematics for arm, differential-drive for homebot, visualize lidar, camera, bno055,and motors in rviz
  - fix dockerfile(image on dockerhub works), run both launch files to get full sub-pub arch
  - add camera to docker, maybe get stereo camera
  - fix tennis navigation
  - make a better physical build, make your own lidar base link,replace mpu6050 with bno055, encoder equipped tt motors
2. using pytorch course make an unsupervised grasping model, use jetson orin(check out study material on nvidia(test with olama))
3. try networking between computer, arm, and homebot use ros_ip. Look into networking and firewalls and other security features.   
4. add flutter app
5. autocad or freecad for designing: drone version, pcb board


[CODE LINK](https://github.com/canizalesjaime/homebot)


## Credits
The robotic arm shown in the video above, is 3D printed using a model created by [FABRI_CREATOR](https://cults3d.com/en/users/FABRI_CREATOR).  
STL file available here: [Mini Robotic Arm on Cults3D](https://cults3d.com/en/3d-model/gadget/mini-robotic-arm)  
Licensed under Cults PU (Personal Use) – No commercial use or AI applications.
