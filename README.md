# ROS2-Open3D-PointCloud-Converter
This repo contains a set of functions to convert between sensor_msgs.msg.PointCloud2 and o3d.geometry.PointCloud

I made this code snippet as a helper function for lidar preprocessing node for [UHRacing Autonomous](https://uk.linkedin.com/company/uh-racing-autonomous). I was searching for a converter but I didn't find one that was working in ros-galactic, hence lead to the creation of these functions. I have posting it in my git so that anyone who faces the same issue in the future can use it.

Tested on ros-galactic and ros-foxy with Open3D version 0.13.0.

If you see any form of improvement feel free to create an issue and then a pull request.
