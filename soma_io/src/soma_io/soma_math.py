from geometry_msgs.msg import Pose, Quaternion


def quaternion_to_matrix(quaternion_msg):
    """
    Convert ROS quaternion_msg to 3 by 3 rotation matrix.
    :param quaternion_msg:
    :return:
    """
    rot = []
    if isinstance(quaternion_msg, Quaternion):
        a11 = 1 - 2 * pow(quaternion_msg.y, 2) - 2 * pow(quaternion_msg.z, 2)
        a12 = 2 * quaternion_msg.x * quaternion_msg.y - 2 * quaternion_msg.z * quaternion_msg.w
        a13 = 2 * quaternion_msg.x * quaternion_msg.z + 2 * quaternion_msg.y * quaternion_msg.w
        a21 = 2 * quaternion_msg.x * quaternion_msg.y + 2 * quaternion_msg.z * quaternion_msg.w
        a22 = 1 - 2 * pow(quaternion_msg.x, 2) - 2 * pow(quaternion_msg.z, 2)
        a23 = 2 * quaternion_msg.y * quaternion_msg.z - 2 * quaternion_msg.x * quaternion_msg.w
        a31 = 2 * quaternion_msg.x * quaternion_msg.z - 2 * quaternion_msg.y * quaternion_msg.w
        a32 = 2 * quaternion_msg.y * quaternion_msg.z + 2 * quaternion_msg.x * quaternion_msg.w
        a33 = 1 - 2 * pow(quaternion_msg.x, 2) - 2 * pow(quaternion_msg.y, 2)
        rot.append([a11, a12, a13])
        rot.append([a21, a22, a23])
        rot.append([a31, a32, a33])
    return rot

