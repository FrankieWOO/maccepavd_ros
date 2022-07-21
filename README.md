# maccepavd_ros
Control software for maccepavd robot based on ROS

Repositories:
- https://github.com/FrankieWOO/maccepavd_ros
- https://github.com/FrankieWOO/mccp_dynamixel_sdk
- https://github.com/FrankieWOO/mccp_dxl_msgs
- https://github.com/FrankieWOO/mccp_dxl_toolbox
- https://github.com/FrankieWOO/mccp_dynamixel_workbench_controllers

to run MACCEPA-VD robot, 
1. roslaunch the dynamixel_workbench_controllers's launch file, 
2. roslaunch the maccepavd_ros's launch file. 

The codes for experiments are in the python code folder of maccepavd_ros. The other 4 repos are forked from Dynamixel, Robotis, to control Dynamixel servomotors; I made some changes so I forked the repos. 

In addition, in the maccepavd_ros repo, there's a branch 'hitec' which was used for controlling hitec servomotors (PWM controlled servos)
