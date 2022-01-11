# Ros2_Project1_Train_Your_Turtle
## You just noticed your cutie cutie turtle is way too fat that he needs to go on a strict diet. Make him run faster than Bolt!

---

# Version
## OS
Ubuntu 18.04 LTS
## Ros2
Ros2-Dashing

---

# Description
## Diagram
/circular_motion(Topic) -> /trainer(Node) -> /turtle1/cmd_vel(Topic) -> /turtlesim(Node)  
An rqt-graph will be added. Sry im too lazy :D

## Topic
/circular motion  
- float32 radius
- float32 linear_velocity
- bool clockwise

---

# How to Use
## First, you do
    git clone https://github.com/Gonnnnn/Ros_Project.git

And then move 'msg_pkg' and 'yeet_go_run_my_turtle' to src directory under your workspace(commonly, it's ~/robot_ws)

    cd ./Ros2_Project1_Train_Your_Turtle
    mv /msg_pkg ~/{workspace}/src
    mv /yeet_go_run_my_turtle ~/{workspace}/src

And do the following

    cd ~/{workspace}
    colcon build --symlink-install --packages-select msg_pkg yeet_go_run_my_turtle

## Run each node
Run turtlesim_node and trainer. You can make your turtle will run with the linear speed of 3.0m/s making a circle with a radius of 3.0m in the clockwise direction through the following example lines.

    ros2 run turtlesim turtlesim_node
    ros2 run yeet_go_run_my_turtle trainer -r=3.0 -v=5.0 -cw=t

## Launch a launch file
You can train your turtle by executing launch file as well. In this case, you'll have to publish the topic(/circular_motion) like the following example lines.

    ros2 launch yeet_go_run_my_turtle train_my_turtle.launch.py
    ros2 topic pub --once /circular_motion msg_pkg/msg/RVD "{radius: 3.0, linear_velocity: 5.0, clockwise: True}"

## If it doesn't work?
welp.. :D

---

# Reference
## Naver Cafe : Open Source Robot Cafe(오로카)
7 Package and node
https://cafe.naver.com/openrt/24065

24 Python - Basic Programming
https://cafe.naver.com/openrt/24450

27 Topic interface
https://cafe.naver.com/openrt/24629

28 Python package building
https://cafe.naver.com/openrt/24637

29 Topic programming
https://cafe.naver.com/openrt/24644

32 Python – parameter
https://cafe.naver.com/openrt/24690

33 Python – argment
https://cafe.naver.com/openrt/24734

40 Launch file
https://cafe.naver.com/openrt/24735
