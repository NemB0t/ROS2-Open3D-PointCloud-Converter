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

# Reference: https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo/blob/master/pcd_demo/pcd_publisher/pcd_publisher_node.py
def O3DPointCloud_to_ROSpc2(open3d_cloud, frame_id="map"):
    """ Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions.
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message

    Code source:
        https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0

    References:
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
        http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html

    """
    # In a PointCloud2 message, the point cloud is stored as an byte
    # array. In order to unpack it, we also include some parameters
    # which desribes the size of each individual point.
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.
    np_open3d_cloud = np.asarray(open3d_cloud.points)
    data = np_open3d_cloud.astype(dtype).tobytes()

    # The fields specify what the bytes represents. The first 4 bytes
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [PointField(
        name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    # The PointCloud2 message also has a header which specifies which
    # coordinate frame it is represented in.
    header = Header(frame_id=frame_id)

    return PointCloud2(
        header=header,
        height=1,
        width=np_open3d_cloud.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3),  # Every point consists of three float32s.
        row_step=(itemsize * 3 * np_open3d_cloud.shape[0]),
        data=data
    )