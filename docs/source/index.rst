Arm Demos Documentation
=======================

.. toctree::
   :maxdepth: 1
   :hidden:

   Installation
   Camera Connection
   Development
   Usage

Arm Demos is a ROS 2 project to demonstrate the capabilities of the UR3e robotic arm.
The goal of this project is to have fun, safe, and interactive robotics demonstrations, as well as to provide a working baseline for future development with the UR3e.

Features
--------

* Robot motion planning with inverse kinematics and collision avoidance using MoveIt, TRAC-IK, and OMPL.
* Human hand motion and gesture imitation using Google MediaPipe Hand Landmarker and OpenCV.

Libraries
---------

* ROS 2: Node system for programming robots
* MoveIt 2 (TRAC-IK, OMPL): Motion planning
* DepthAI SDK: Camera streaming
* Google MediaPipe Hand Landmarker: Hand video to joint coordinates
* OpenCV: Camera stream and landmark view

Requirements
------------

* A host computer.
* A Universal Robots UR3e robotic arm, connected with ethernet.
* A Luxonis OAK-D Pro camera, connected with high-speed USB3.

Getting Started
---------------

* :doc:`Installation <Installation>`

  - Instructions to set up the arm demos repository

* :doc:`Camera Connection <Camera Connection>`

  - Instructions to connect to an OAK-D camera for hand tracking

* :doc:`Development <Development>`

  - Instructions for working in the project's development environment

* :doc:`Usage <Usage>`

  - Instructions to run the included tools and demos
