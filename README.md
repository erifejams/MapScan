# MapScan

# Using 
• Python <br>
• CoppeliaSim and the simulated MyT <br>
• ROS2 <br>
• Linux <br>

# Project Description
The goal of this project is to detect aruco markers using the robot when it moves around the room.


# To run 
## Terminal #1:
### (EVERYTIME CHANGES ARE MADE TO THE PACKAGE CD TO THE ~/dev_ws/ folder): <br>
•	cd ~/dev_ws <br>
• colcon build <br>
•	source ~/dev_ws/install/setup.bash <br>

## Terminal #2:
### to get the coppelia sim running
cd ~/apps/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04<br>
bash coppeliaSim.sh <br>
load the scene that can be found in scene/scene.ttm <br>
press the start button to start the simulation <br>


## Terminal #3:
source ~/dev_ws/install/setup.bash <br>
ros2 launch thymioid main.launch device:="tcp:host=localhost;port=33333" simulation:=True name:=thymio0


## Terminal #4:
### To run the compulsary.launch.xml (for task 3):
• source ~/dev_ws/install/setup.bash <br>
• ros2 launch MapScan controller.launch.xml thymio_name:=thymio0 <br>


## Terminal #5:
#### to see the visualisation and camera from the robot pov
ros2 topic list (to see the topics of the robot - thymio0) <br>
rqt
