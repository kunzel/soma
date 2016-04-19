import octomap
import soma_math
import numpy as np
from octomap_msgs.msg import Octomap
from geometry_msgs.msg import Pose, PoseStamped


class SOMAOctree(object):

    def __init__(self, octr_path=None, res=0.01):
        # init octree and load data
        self.octree = octomap.OcTree(res)
        if octr_path is None:
            return
        if not self.octree.readBinary(octr_path):
            raise Exception('failed when reading octomap binary file')

    def load_tree(self, input_path):
        """
        Loading octree info.
        :param input_path: the whole path of the octomap file
        :return: BOOL return false if failed
        """
        return self.octree.readBinary(input_path)

    #TODO: define a octomap bt format template
    def load_tree_from_msg(self, oct_msg):
        self.oct_template = '# Octomap OcTree binary file\n'
        self.oct_template += 'id ' + db_query_result[i][0].id + '\n'
        self.oct_template += 'res ' + str(db_query_result[i][0].resolution) + '\n'
        self.oct_template += 'data\n'
        if oct_msg is Octomap:
            pass
        else:
            pass

    def save_tree(self, p, binary=True):
        if binary:
            self.octree.writeBinary(p)
        else:
            self.octree.write(p)

    @property
    def bbx_info(self):
        return self.get_bbx_info()

    def get_bbx_info(self):
        bbx = {'min': self.octree.getMetricMax(),
               'max': self.octree.getMetricMin(),
               'size': self.octree.getMetricSize()}
        return bbx

    @property
    def transformed_bbx_info(self):
        return self.get_bbx_info()

    def get_transformed_bbx(self):
        bbx = self.get_bbx_info()

    def to_global(self, transform, octo_tree=None):
        """
        Apply transformation to octomap octree (rotation first)
        :param octo_tree:
        :param transform: ROS Pose or PoseStamped info
        :return:
        """
        if isinstance(transform, PoseStamped) or isinstance(transform, Pose):
            rot_mat = soma_math.quaternion_to_matrix(transform.orientation)
            max_p = np.dot(rot_mat, [[self.bbx_info['max'][0]], [self.bbx_info['max'][1]], [self.bbx_info['max'][2]]])
            max_p[0] += float(transform.pose.position.x)
            max_p[1] += float(transform.pose.position.y)
            max_p[2] += float(transform.pose.position.z)

            min_p = np.dot(rot_mat, [[self.bbx_info['min'][0]], [self.bbx_info['min'][1]], [self.bbx_info['min'][2]]])
            min_p[0] += float(transform.pose.position.x)
            min_p[1] += float(transform.pose.position.y)
            max_p[2] += float(transform.pose.position.z)

        return min_p, max_p

    #TODO: more options for args
    def check_point(self, octo_tree=None, *args):
        """
        check if a point is in the octree covered space
        :param octo_tree: an octomap octree instance
        :param args: support tow types 1. three number 2. an Posr msg instance
        :return: True when the point is in bbx
        """
        if octo_tree is None:
            _min = self.octree.getMetricMax()
            _max = self.octree.getMetricMin()
        else:
            _min = octo_tree.getMetricMax()
            _max = octo_tree.getMetricMin()

        if len(args) is 3:
            _x = args[0]
            _y = args[1]
            _z = args[2]
            if _min[0] < _x < _max[0] and \
               _min[1] < _y < _max[1] and \
               _min[2] < _z < _max[2]:
                return True
            else:
                return False

        if isinstance(args[0], Pose):
            _x = args[0].position.x
            _y = args[0].position.y
            _z = args[0].position.z
            if _min[0] < _x < _max[0] and \
               _min[1] < _y < _max[1] and \
               _min[2] < _z < _max[2]:
                return True
            else:
                return False
        raise Exception("unknown args type or amount of args.")
