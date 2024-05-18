import open3d as o3d
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField


def normalize_np(array, alpha=0, beta=65535):
    # Calculate min and max values of the array
    # This function is made to remove dependency on cv2
    # The function simulates cv2.normalize(array, dst=None, alpha=0, beta=65535, norm_type=cv2.NORM_MINMAX)
    min_val = np.min(array)
    max_val = np.max(array)

    # Normalize the array to the range [alpha, beta]
    normalized_array = (array - min_val) / (max_val - min_val) * (beta - alpha) + alpha

    return normalized_array

# Reference: https://github.com/felixchenfy/open3d_ros_pointcloud_conversion/blob/master/lib_cloud_conversion_between_Open3D_and_ROS.py
def ROSpc2_to_O3DPointCloud(pc2):

    xyz,colors,min_intensity,max_intensity = ROSpc2_to_nparray(pc2)
    return nparray_to_O3DPointCloud(xyz,colors),min_intensity,max_intensity#3 cause xyz coordinates and intensity is 1


def ROSpc2_to_nparray(pc2):
    # we are assuming all the elements in the sensor_msgs.msg.PointField are having same datatype
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
    original_dtype = [(pc2.fields[i].name, _PCL2_DATATYPES_NUMPY_MAP[pc2.fields[i].datatype]) for i in range(len(pc2.fields))]
    recovered_array = np.frombuffer(pc2.data, dtype=original_dtype)
    # reshaping into a 1d array of points where each point is an array containing x y z intersity(if applicable)

    if len(recovered_array) == 0:
        print("Converting an empty cloud")
        return None

    intensity_index = -1
    x_index = y_index = z_index = -1
    for i in range(len(pc2.fields)):
        if pc2.fields[i].name == 'x':
            x_index = i
            continue
        if pc2.fields[i].name == 'y':
            y_index = i
            continue
        if pc2.fields[i].name == 'z':
            z_index = i
            continue
        if pc2.fields[i].name == 'intensity':
            intensity_index = i
            continue
    intensity=[]
    data = np.array(recovered_array.tolist())
    if intensity_index!=-1:
        data = np.concatenate((data[:, x_index].reshape(-1, 1),data[:, y_index].reshape(-1, 1),data[:, z_index].reshape(-1, 1), data[:, intensity_index].reshape(-1, 1)), axis=1)
        data = data[~np.isnan(data)].reshape(-1, 4)
        xyz = data[:, :3]
        intensity = data[:, -1]
    else:
        xyz = data[~np.isnan(data)].reshape(-1, 3)
    colors=[]
    min_intensity = max_intensity = -1
    if intensity_index != -1:#Reference: abhishekbawkar's comment on https://github.com/isl-org/Open3D/issues/921
        # If the intensity array is RGB encoded, first normalize it using opencv to 16-bit precision
        min_intensity = np.min(intensity)
        max_intensity = np.max(intensity)
        intensities_norm = normalize_np(intensity,0,65535)
        # Using numpy.column_stack() provide equal values to RGB values and then assign to 'colors' property
        # of the point cloud.
        # Since Open3D point cloud's 'colors' property is float64 array of shape (num_points, 3), range [0, 1],
        # we have to normalize the intensity array by dividing it by 65535
        colors = np.column_stack((intensities_norm, intensities_norm, intensities_norm)) / 65535
    return xyz,colors,min_intensity,max_intensity

def nparray_to_O3DPointCloud(xyz,colors=-1):
    open3d_cloud = o3d.geometry.PointCloud()

    # combine
    open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))  # numpy to o3d

    #Reference: abhishekbawkar's comment on https://github.com/isl-org/Open3D/issues/921
    if len(colors)!=0 or colors!=-1:
        open3d_cloud.colors = o3d.utility.Vector3dVector(colors)

    return open3d_cloud

# Reference: https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo/blob/master/pcd_demo/pcd_publisher/pcd_publisher_node.py
def O3DPointCloud_to_ROSpc2(open3d_cloud, frame_id="map",min_intensity=-1,max_intensity=-1):
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
    np_open3d_cloud_coordinates,np_open3d_cloud_intensities=O3DPointCloud_to_nparray(open3d_cloud,min_intensity,max_intensity)
    return nparray_to_ROSpc2(np_open3d_cloud_coordinates, frame_id,np_open3d_cloud_intensities)


def O3DPointCloud_to_nparray(open3d_cloud,min_intensity=-1,max_intensity=-1):
    # In a PointCloud2 message, the point cloud is stored as an byte
    # array. In order to unpack it, we also include some parameters
    # which desribes the size of each individual point.

    np_open3d_cloud_coordinates = np.asarray(open3d_cloud.points)
    colors=np.asarray(open3d_cloud.colors)
    intensities_65535 = (colors[:, 0] * 65535).astype(np.float32)
    np_open3d_cloud_intensities = np.ceil((intensities_65535 / 65535.0) * (max_intensity - min_intensity) + min_intensity)
    return np_open3d_cloud_coordinates.reshape(-1,3),np_open3d_cloud_intensities.reshape(-1,1)
def nparray_to_ROSpc2(np_open3d_cloud,frame_id="map",np_open3d_cloud_intensities=[]):
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

    # The fields specify what the bytes represents. The first 4 bytes
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    if len(np_open3d_cloud_intensities)>0:
        fields = [PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
                  for i, n in enumerate(['x','y','z','intensity'])]
        np_open3d_cloud=np.concatenate((np_open3d_cloud,np_open3d_cloud_intensities),axis=1)
    else:
        fields = [PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
                  for i, n in enumerate('xyz')]
    data = np_open3d_cloud.astype(dtype).tobytes()
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
        point_step=(itemsize * len(fields)),  # Every point consists of three float32s.
        row_step=(itemsize * len(fields) * np_open3d_cloud.shape[0]),
        data=data
    )