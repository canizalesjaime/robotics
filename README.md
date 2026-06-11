# To Do (CONVERT THIS TO DOCUMENTATION)
1. homebot: urdf file, fix tennis navigation, refactor new code to ros2 with urdf files using dht tables for frames, stereo camera, lidar slam for 3d stuff. Make the base rotate, at the same time as being able to receive other commands to arm and wheels.
2. House keeping: database for objects seen with dates(web dev course), automate git push for ci/cd, aws?
3. using pytorch course make an unsupervised grasping model, use jetson orin(check out study material on nvidia(test with olama))
4. try networking between computer, arm, and homebot use ros_ip. Look into networking and firewalls and other security features.    


# later maybe
1. contribute to ros open source
2. add flutter app
3. autocad or freecad for designing: drone, pcb board


# steps to start
1. cd /workspaces/homebot/ros2_ws && colcon build && source install/setup.bash
2. ros2 run robot_control \<executable_name\>
3. ros2 launch robot_control \<executable_name\>
4. ros2 service call /capture_image std_srvs/srv/Trigger
5. xhost +local:root and xhost -local:root for rviz2
6. ros2 launch my_robot_description display.launch.py

**on pi to control motors and stuff** 
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


**on kubuntu for web app(just use dev container on windows)**  
1. docker build -f ./robot_dashboard.Dockerfile -t ros2-setup .
2. docker run -it --rm --net=host --name web-container -v /home/jaime/homebot:/workspace web-setup:latest

## Ultrasonic HCSR04
* VCC -> PIN 2(5V)
* TRIG -> PIN 29 (GPIO 5)
* ECHO -> PIN 31 (GPIO 6) with voltage divider
* GND -> PIN 39 (GND)


## Accelerometer MPU6050 
* VCC -> PCB
* GND -> PIN 6 (GND)
* SCL -> PCB
* SDA -> PCB


## TB6612 Motor Driver 1
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

## PCA9865
* VCC(3.3) -> PCB
* SDA -> PCB
* SCL -> PCB
* GND -> PIN 34

## Lidar
* VCC -> PIN 4(5V)
* RX -> PIN 8 (GPIO 14)
* TX -> PIN 10(GPIO 15)
* GND -> PIN 14
