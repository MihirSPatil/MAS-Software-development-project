# Software Development Project (SS2018) Group 2

## Project 3: 3D object detection and tracking

The Rotating Turntable Test (RTT) in the RoboCup@Work competition requires the detection, tracking and recognition of objects moving on a turntable. An existing scene segmentation component segments objects lying on a planar surface using 3D pointclouds. In order to use this component for RTT, some refactoring and new functionality is required.

## Problem

The following are not problems, but are instead missing requirements in the existing scene segmentation component that would make it suitable for the RTT:

* in it’s current set up, it is not programmed for dynamic scenes (i.e. it performs a one-shot segmentation on a static scene)
* has no tracking component
* has nothing that exploits the ”circular” nature of the motion of the objects
* cannot estimate speed and motion path of the segmented objects

### Immediate Goals

* refactor to account for the dynamic scene. In particular, the plane needs to be extracted just once, while multiple segmentation and clustering operations can be performed using the
once extracted plane.

### Long-term Goals

Program the missing requirements, including:

* track objects using a naive nearest-neighbour approach
* fit a circle to the tracked objects and estimate their speed and position on the table
* output an estimated time when an object will be in front of the robot for grasping
