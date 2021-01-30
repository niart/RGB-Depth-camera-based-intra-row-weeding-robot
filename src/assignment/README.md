the focus of my assignment is the perception strategies of this field robot. 
This file instructs the launching of this simulator and explains the vision-based perception and navigation algorithm.

install the simulator:
sudo apt-get install ros-melodic-uol-cmp9767m-base

run source /opt/ros/melodic/setup.bash

launch the simuator:
roslaunch uol_cmp9767m_base thorvald-sim.launch

Install the L-CAS Ubuntu distribution as explained here: 
https://github.com/LCAS/rosdistro/wiki#the-l-cas-build-farm
try 
sudo apt-get install ros-melodic-desktop ros-melodic-uol-cmp9767m-base
after you have installed the L-CAS distribution

to find the coordinates of robot:
rostopic echo /thorvald_001/odometry/base_raw

1. To launch the simulation environment, cd into NW_assignment_ws and run:
roslaunch uol_cmp9767m_base thorvald-sim.launch obstacles:=false second_robot:=false fake_localisation:=true

Afterwards, Gazebo will load environment model and robot model

2. To launch the perception and controller node, run:
python src/assignment/run.py 
This node subscribes depth image from topic '/thorvald_001/kinect2_sensor/sd/image_depth_rect' and RGB image from topic '/thorvald_001/kinect2_camera/hd/image_color_rect' synchronously.

Visualize with: rqt_image_view


3. Perception strategies:
I initially intended to use depth information to further distinguish different types of plants, but later it's found that those plants in this simulator are plain picture. Therefore depth image is not dealt,
Before detecting the plants, ros messages are converted to cvImage.
Then detection of 3 types of plants will be carried out based on color and geometric features.
In the sight of onboard camera, "green" zone marks weeds, "blue" marks lettuce, and "red" marks cabbage.

General image processing procedures includes: convert RGB color space into HSV, color segment, erode, dilate.
Another solution was added to distinguish cabbage from weeds, which have very similar color values:
contours detected are to be matched to a minimum enclosing circles and a value "color occupation" will be calculated. If the contour is very close to its enclosing circle, this target will be recognized as cabbage; otherwise, weed. This is because cabbage leaves are more cicular and weeds are narrow.

Next, the detector calculates the middle point of the detected segment to generate waypoints, and command the robot to track it.

4. Navigation strategies:
The robot is initially navigated by pre-defined trajectory to start inspection. However, after the targets come into sight, the perceived information will instructs change of threshold values and control command. Waypoint tracking will generate the linear speed and angler speed to control robot moving along with the plants rows. 

5. Control:
To control robot moved in gazebo, Publish control command to the topic '/thorvald_001/turtle1/cmd_vel'. Its message type is Twist.
