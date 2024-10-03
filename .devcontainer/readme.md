# Steps for Docker:
1. If image is not built , build it by running:
        docker build -t fetch-gazebo_melodic .
2. Make sure you are cd in repo you want to mount!
   * **Windows\(has gui\):** In xlaunch make display number 0 , and for additional parameters add -nowgl. Then from powershell run: docker run -v .:/root/catkin_ws/src/apollo_detection_systems -w /root/catkin_ws/src/apollo_detection_systems -it --name r1_melodic -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 fetch-gazebo_melodic 
   * **Linux or Mac:** sudo docker run -v ${PWD}/:/root/catkin_ws/src/apollo_detection_systems -w /root/catkin_ws/src/apollo_detection_systems -it --name r1_melodic fetch-gazebo_melodic
   * **No mounted repo:** sudo docker run -w /workspace -it --name r1_melodic fetch-gazebo_melodic
3. **Connect to Robot:** Use the same commands above for your os, but add the commandline argument **--net=host** before commandline arugment **fetch-gazebo_melodic**. 
4. **Connect to Robot:** Once container is open run: sudo echo "192.168.1.5 fetch30-MS-7851" >> /etc/hosts
5. **Connect to Robot:** On apollo(robot) make sure to add your computers ip address to the /etc/hosts on robot. 

# Docker commands to remember:
1. docker ps -a (Shows container ids)
2. docker commit \<containerID\> \<repository\>:\<tag\> (to save changes on container to image)
3. docker rm \<containerID\> (delete container)
4. docker exec -it \<containerID\> bash (open another terminal in a current container)


## Notes:
1. If you are running on windows computer, use vim run :set ff=unix then :wq
2. catkin_make repo since you are mounting it.
3. Only windows version has access to gui in docker. 
4. **You can also open in dev container to make above easier**. in vscode: reopen container at top prompt.
