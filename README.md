# Project Description 
* A compact 3D-printed mobile robot with a 4-DOF servo arm and dual TT motors for navigation.
Integrated LiDAR for obstacle avoidance, a camera with YOLOv8n for real-time object detection, and an MPU6050 IMU for pose estimation.  
* A web-based interface for remote control of the robot and arm, featuring
live video with detection overlays and accelerometer data. Implemented using 
react and tailwind css to design the frontend and fast api to interact with
the backend.


# System Design

## Software architecture
Main creates an object for each peripheral device which has a class defined: motor wheels, arm, accelerometer, camera, and lidar. Depending on feedback from frontend or physical sensors, call relevant method to determine output. One dockerfile sets up frontend for use on your computer, and
another dockerfile sets up dependencies on raspberry pi 5 to run backend.<br>
<img src="./content/images/software_architecture.png" width ="500" >


## Hardware
* stl files used for 3D printing robot base included in [freecad_files](./free_cad_files)
* pcb board power by a 7.4V LiPo battery 2200mAh, buck converter to 5V to power raspberry pi 5, 7.4V used to power all the motors(4 servo, 2 tt motors)

### Peripheral Sensors, Motor Drivers Used on Raspberry pi 5
#### Ultrasonic HCSR04
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
1. cd /workspaces/homebot/ros2_ws && colcon build && source install/setup.bash
2. ros2 run robot_control \<executable_name\>
3. ros2 launch robot_control \<executable_name\>
4. ros2 service call /capture_image std_srvs/srv/Trigger
5. xhost +local:root and xhost -local:root for rviz2
6. ros2 launch my_robot_description display.launch.py


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


# To Do
1. homebot: slam/urdf file use dht table for frames,  Make the base rotate, at the same time as being able to receive other commands to arm and wheels, stereo camera, fix tennis navigation
2. House keeping: database for objects seen with dates(web dev course), automate git push for ci/cd, aws?
3. using pytorch course make an unsupervised grasping model, use jetson orin(check out study material on nvidia(test with olama))
4. try networking between computer, arm, and homebot use ros_ip. Look into networking and firewalls and other security features.
5. make a better physical build (hdmi exposed)    
6. add flutter app
7. autocad or freecad for designing: drone version, pcb board
8. Add robot image here