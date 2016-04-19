#!/usr/bin/env python
import rospy
from soma_io.soma_io import FileIO
from soma_io.state import World
from soma_io.octree import SOMAOctree
from soma_io import soma_math
import numpy as np


if __name__ == '__main__':
    ROMBUS_DB = "/media/psf/strands_data_backup/20150505/patrol_run_10/room_6"
    object_classes = FileIO.scan_objects(ROMBUS_DB)

    rospy.init_node("octreeINFO_importer", anonymous=True)

    world = World('octree_info')

    bt_files = FileIO.scan_file(ROMBUS_DB, '.bt')

    for bt_file in bt_files:
        bbx_info_list = []
        # load octomap data
        octr = SOMAOctree()
        if octr.load_tree(bt_file):
            bbx_info = octr.bbx_info
        else:
            raise Exception('failed when')

        # transform to global coordinate
        file_name = bt_file[:-3]+'.pcd'
        xml_file = file_name[0:file_name.rfind('/')+1] + 'room.xml'
        file_name = file_name[file_name.rfind('/')+1:len(file_name)]
        # only these octomap file are considered
        if not file_name.startswith('intermediate_'):
            continue

        gl_pose = FileIO.get_xml_pose(xml_file, file_name)
        rot_mat = soma_math.quaternion_to_matrix(gl_pose.pose.orientation)

        max_p = np.dot(rot_mat, [[bbx_info['max'][0]], [bbx_info['max'][1]], [bbx_info['max'][2]]])
        max_p[0] += float(gl_pose.pose.position.x)
        max_p[1] += float(gl_pose.pose.position.y)
        max_p[2] += float(gl_pose.pose.position.z)

        min_p = np.dot(rot_mat, [[bbx_info['min'][0]], [bbx_info['min'][1]], [bbx_info['min'][2]]])
        min_p[0] += float(gl_pose.pose.position.x)
        min_p[1] += float(gl_pose.pose.position.y)
        min_p[2] += float(gl_pose.pose.position.z)

        bbx_info_list.append(list(bbx_info['max']))
        bbx_info_list.append(list(bbx_info['min']))

        for i in xrange(0, 3):
            bbx_info_list[0][i] = float(max_p[i])
            bbx_info_list[1][i] = float(min_p[i])

        # store in database
        db_octr = world.create_object(bt_file)
        db_octr.add_bbx(bbx_info_list)

