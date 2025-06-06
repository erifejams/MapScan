# Finder

# Technologies
• Python <br>
• CoppeliaSim and the simulated MyT <br>
• ROS2 <br>
• Linux <br>
• Pytorch<br>


# Project Description
Goal One: The goal of this project is for mighty thymio to randomly explore the room without colliding with objects, with the goal of finding a specific item using aruco markers <br>
Goal Two: When the specific item is found, another mighty thymio creates a 3D reconstruction of the object.<br><br>


**NOTES** <br>
•	Use scene in scene/two_thymio.ttt. (code will still work if u remove thymio_1 to test the part of obstacle avoidvance since with both with id detecting, it doesnt work as well to avoid obstacle + can test obstacle avoidance with 2 thymio, just change the location of thymio 1, so that an id is not detected in the camera)
•	Delete the second mighty thymio from the scene to achieve the first goal. <br>
•	Add the second mighty thymio back to the scene to achieve the second goal. <br>

# Additional Installation
pip install pyrealsense2 open3d <br>
pip install torch torchvision torchaudio <br>
pip install transformers <br>


# To run 
## Terminal #1:
### (EVERYTIME CHANGES ARE MADE TO THE PACKAGE CD TO THE ~/dev_ws/ folder): <br>
•	cd ~/dev_ws <br>
• colcon build <br>
•	source ~/dev_ws/install/setup.bash <br>


## Terminal #2:
### to get the coppelia sim running
•	cd ~/apps/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04<br>
•	bash coppeliaSim.sh <br>
•	load the scene that can be found in scene/scene.ttm <br>
•	press the start button to start the simulation <br>


## Terminal #3:
•	cd ~/dev_ws
•	source ~/dev_ws/install/setup.bash <br>
•	ros2 launch thymioid main.launch device:="tcp:host=localhost;port=33333" simulation:=True name:=thymio0 single:=False  <br>


## Terminal #4:
### To run the compulsary.launch.xml:
•	cd ~/dev_ws/MapScan<br>
• source ~/dev_ws/install/setup.bash <br>
• ros2 launch MapScan controller.launch.xml thymio_name:=thymio_0 thymio_name_two:=thymio_1<br>


## Terminal #5:
#### to see the visualisation and camera from the robot pov
•	ros2 topic list (to see the topics of the robot - thymio_0 and thymio_1) <br>
•	rqt (change between /thymio_1/camera and /thymio_0/camera)

# Demo:
[![Watch the video](MapScan/scanRoom/output.png)](https://youtu.be/2K5-N__InQI)

