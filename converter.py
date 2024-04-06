import open3d as o3d
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField

# Reference: https://github.com/felixchenfy/open3d_ros_pointcloud_conversion/blob/master/lib_cloud_conversion_between_Open3D_and_ROS.py
def ROSpc2_to_O3DPointCloud(pc2):
    #we are assuming all the elements in the sensor_msgs.msg.PointField are having same datatype
    # Reference: https://github.com/dheera/rosboard/blob/main/rosboard/compression.py
    _PCL2_DATATYPES_NUMPY_MAP = {
        1: np.int8,
        2: np.uint8,
        3: np.int16,
        4: np.uint16,
        5: np.int32,
        6: np.uint32,
        7: np.float32,
        8: np.float64,
    }
    original_dtype = _PCL2_DATATYPES_NUMPY_MAP[pc2.fields[0].datatype]
    recovered_array = np.frombuffer(pc2.data, dtype=original_dtype)
    # reshaping into a 1d array of points where each point is an array containing x y z intersity(if applicable)
    recovered_array = recovered_array.reshape(pc2.width, len(pc2.fields))
    #Currently the recovered_array contains array of all points in from the point cloud

    open3d_cloud = o3d.geometry.PointCloud()
    if len(recovered_array) == 0:
        print("Converting an empty cloud")
        return None
    if len(pc2.fields) ==4: #TODO: Add code to handle intensity
        xyz = [(x, y, z) for x, y, z, intensity in recovered_array]
    elif len(pc2.fields) ==3:
        xyz = [(x, y, z) for x, y, z in recovered_array]
    else:
        print("array of size"+len(pc2.fields)+" is not handled")

    # combine
    open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz)) #numpy to o3d

    return open3d_cloud