# Autonomous Small Waste Collection Robot
This repostory contains a ROS Noetic package for camera-feed control of the Unitree Go1 Quadraped.

## The Project
This package is part of the MREN403 Mechatronics and Robotics Design Capstone course at Queen's University. This is a year-long project to test the engineering skills of final-year undergraduate students. 
Our project goal is to design an robot capable of autonomous litter collection around the Queen's Campus and downtown Kingston core.

## Deployment
1. To start run 'ros2 run autonomous_litter_bot_package image_sender' on the pi5
  ! make sure to update the ip address !
2. On the 'offboard-cv' branch of this repo, run 'can_detector.py' on your PC
3. run main.launch on pi4 (double checking ip addresses for reciever node) on the 'smach' branch of this repo

## Structure

## Authors
Nathan Duncan (20ntd1@queensu.ca) - Lead of mobile platform (Go1)
Connall Milberry () - Lead of Computer Vision
Daniel Poon () - Lead of Manipulation
Daniel Dubinko () - Lead of Powersystems and Integration
