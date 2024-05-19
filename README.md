# ROS2-Open3D-PointCloud-Converter

This repo contains a set of functions to convert between sensor_msgs.msg.PointCloud2 and o3d.geometry.PointCloud. This works on all versions of ROS2.

To use the converter, create a converter object.
```python
converter = Converter()
```
Below are the supported conversion:
#### sensor_msgs.msg.PointCloud2 -> o3d.geometry.PointCloud
The function is called "**ROSpc2_to_O3DPointCloud**". 
**Input: **sensor_msgs.msg.PointCloud2
**Output:** o3d.geometry.PointCloud

```python
O3DPC = converter.ROSpc2_to_O3DPointCloud(PC2)
```
#### sensor_msgs.msg.PointCloud2 -> numpy.array
The function is called "**ROSpc2_to_nparray**". 
**Input: **sensor_msgs.msg.PointCloud2
**Output:** o3d.geometry.PointCloud.points,o3d.geometry.PointCloud.colors,min intensity,max intensity
**Note1: **The min intensity and max intensity are the lowest and highest values of the pointcloud intensity which will stored in the object and later used for converting back to Pointcloud2.
**Note2:** The o3d.geometry.PointCloud.points and o3d.geometry.PointCloud.colors are outputed as numpy arrays.
```python
nparray,color,min_intensity,max_intensity = converter.ROSpc2_to_nparray(PC2)
```
#### numpy.array -> o3d.geometry.PointCloud
The function is called "**nparray_to_O3DPointCloud**". 
**Input: ** o3d.geometry.PointCloud.points,o3d.geometry.PointCloud.colors
**Output:** o3d.geometry.PointCloud
**Note1: **The o3d.geometry.PointCloud.points and o3d.geometry.PointCloud.colors should be inputed as numpy arrays. (Refer [Link](http://https://www.open3d.org/docs/0.9.0/tutorial/Basic/working_with_numpy.html "Link") for details)
```python
O3DPC = converter.nparray_to_O3DPointCloud(xyz,color)
```
#### o3d.geometry.PointCloud -> sensor_msgs.msg.PointCloud2
The function is called "**O3DPointCloud_to_ROSpc2**". 
**Input: ** o3d.geometry.PointCloud,frame id,min intensity, max intensity
**Note1: ** The frame ID is the reference frame used by ROS by default it will be map.
**Note2:** The min intensity and max intensity will be stored in the `converter` object if the object was used to convert the data from **"sensor_msgs.msg.PointCloud2 -> o3d.geometry.PointCloud"**. Please pass the min and max intensity values or the converted will assume 0 and 65535 by default.
**Output:** sensor_msgs.msg.PointCloud2

```python
PC2 = converter.ROSpc2_to_O3DPointCloud(O3DPC,"velodyne",min_intensity,max_intensity)
```
**Note3:** Here the frame ID is given as `"velodyne"` since I was using it with Velodyne puck lite LIDAR
#### o3d.geometry.PointCloud -> numpy.array
The function is called "**O3DPointCloud_to_nparray**". 
**Input: ** o3d.geometry.PointCloud,min intensity, max intensity
**Output:** xyz coordinates , intensity
**Note1: ** The xyz coodinates are in the shape (N,3) and intensity is in the shape (N,1) where N is the number of lidar points in the data.
```python
xyz,intensity = converter.O3DPointCloud_to_nparray(O3DPC,min_intensity,max_intensity)
```
#### numpy.array -> sensor_msgs.msg.PointCloud2
The function is called "**nparray_to_ROSpc2**". 
**Input: ** xyz coordinates, frame id, intensity
**Output:** sensor_msgs.msg.PointCloud2
```python
PC2 = converter.nparray_to_ROSpc2(xyz,"velodyne",intensity)
```
**Note1: ** The xyz coodinates must be in the shape (N,3) and intensity must be in the shape (N,1) where N is the number of lidar points in the data.

I made this code snippet as a helper function for lidar preprocessing node for  [UHRacing Autonomous](https://uk.linkedin.com/company/uh-racing-autonomous).  I was searching for a converter but I didn't find one that was working in ros-galactic,  hence lead to the creation of these functions.  I am posting it in my github so that anyone who faces the same issue in the future can use it.

Tested on ros-galactic and ros-foxy with Open3D version 0.13.0.

If you see any form of improvement feel free to create an issue and then a pull request.

