# slam3d-ros

## Overview
This package provides a basic ROS integration for the SLAM3D mapping library.
It contains a simple mapper that uses 3D-Laser-Scans and Odometry to create
Pointcloud-Maps of a mobile robots surroundings together with some helper
functions to create own mapping applications based on SLAM3D.

## Installation
The following section assumes that the reader is familiar with the usage of
the ROS eco-system. The library SLAM3D is a plain CMake-Package (non-catkin),
which is normally handled by catkin-python. If you don't use these, you
can either use catkin_make_isolated to build within your workspace or install
SLAM3D to your system with cmake/make.

 - apt install python-catkin-tools

Checkout slam3d and this package into your ROS workspace.
 - https://github.com/dfki-ric/slam3d
 - https://github.com/dfki-ric/slam3d-ros

For better performance, activate release-build and static libs. If you want
to use shared libraries set the parameter to one, but do not leave it
unspecified.
 - catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=0

Build your workspace.
 - catkin build

## Getting started
To get an idea how to build your own mapping application with SLAM3D you might
want to have a look at src/kitti_mapper.cpp for a minimalistic example. It only
uses Pointclouds without Odometry to perform mapping based on sequential scan
matching. A more sophisticated implemmentation is the somewhat generic
src/mapper_node.cpp, which also uses Odometry (via tf) and GPS localization for
a much more robust SLAM solution.
