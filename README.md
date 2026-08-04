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
* pcb board power by a 7.4V LiPo battery 2200mAh, buck converter to 5V to power raspberry pi 5 and 4 servo motors, 7.4V used to power all the tt motors(with encoders)
* The robotic arm shown in was 3D printed using a model created by [FABRI_CREATOR](https://cults3d.com/en/users/FABRI_CREATOR).  
STL file available here: [Mini Robotic Arm on Cults3D](https://cults3d.com/en/3d-model/gadget/mini-robotic-arm)  
Licensed under Cults PU (Personal Use) – No commercial use or AI applications.


### Peripheral Sensors, Motor Drivers Used on Raspberry pi 5
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

#### PCA9685 12-bit PWM controller
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

## Backend(must run on raspberry pi)  
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
  -v /home/jaime/homebot:/workspaces/homebot \
  jaimec21/jazzy_pi:latest
3. Testing backend without ros2:
```
uvicorn main:app --host 0.0.0.0 --port 8000
```

### workspace setup(backend with ros2)
1. source /opt/jazzy/setup.bash
2. cd /workspaces/homebot/ros2_ws
3. colcon build or or colcon build --symlink-install
4. source install/setup.bash
5. ros2 launch robot_control robot_move_launch.py
6. ros2 launch my_robot_description display.launch.py


### useful commands to remember
1. ros2 service call /capture_image std_srvs/srv/Trigger
2. xhost +local:root and xhost -local:root (for rviz2 on kubuntu)
3. ros2 run tf2_tools view_frames (saves tf frames in pdf)
4. ros2 pkg create my_robot_description --build-type ament_python  (create package)
5. ros2 run rqt_graph rqt_graph (view sub-pub architecture)
6. ros2 run \<package_name\> \<executable_name\>
7. ros2 launch \<package_name\> \<executable_name\>
8. simple docker:
```
docker run -it --rm --init --privileged --net=host --name ros2-container -v /home/jaime/homebot:/workspaces/homebot jaimec21/jazzy_pi:latest
```
9. open another terminal for container:
```
docker exec -it ros2-container bash
```


# To Do
1. homebot: 
  - lidar, stop base is lagged, dont see lidar points: sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1785800243, nanosec=113936840), frame_id=''), name=['servo_rotation_joint'], position=[2.6179938779914944], velocity=[], effort=[])
sensor_msgs.msg.LaserScan(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1785800241, nanosec=579219579), frame_id='lidar_link'), angle_min=0.5235987755982988, angle_max=2.6179938779914944, angle_increment=0.17453292519943295, time_increment=0.05, scan_time=0.6, range_min=0.04, range_max=4.0, ranges=[4.800000190734863, 4.800000190734863, 4.800000190734863, 4.800000190734863, 4.800000190734863, 4.800000190734863, 4.800000190734863, 4.800000190734863, 4.800000190734863, 4.800000190734863, 4.800000190734863, 4.800000190734863, 4.800000190734863], intensities=[1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0])
  - make rotate_base and stop_base into a service instead(maybe other "actions" as well)
  - slam/urdf file for frames and kinematics for arm, differential-drive for homebot, visualize lidar, camera, bno055,and motors in rviz, fix orientation of lidar in rviz when publishing servo rotation(test solution)
  - fix dockerfile(image on dockerhub works), run both launch files to get full sub-pub arch
  - add camera to docker, maybe get stereo camera
  - fix tennis navigation
  - make a better physical build, make your own lidar base link,replace mpu6050 with bno055, encoder equipped tt motors
  - try networking between computer, and homebot use ros_ip. Look into networking and firewalls and other security features.
2. using pytorch course make an unsupervised grasping model, use jetson orin(check out study material on nvidia(test with olama))   
3. add flutter app
4. autocad or freecad for designing: drone version, pcb board


[CODE LINK](https://github.com/canizalesjaime/homebot)


## Credits
The robotic arm shown in the video above, is 3D printed using a model created by [FABRI_CREATOR](https://cults3d.com/en/users/FABRI_CREATOR).  
STL file available here: [Mini Robotic Arm on Cults3D](https://cults3d.com/en/3d-model/gadget/mini-robotic-arm)  
Licensed under Cults PU (Personal Use) – No commercial use or AI applications.
