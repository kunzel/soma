#!/usr/bin/env python
import rospy
import octomap
from soma_io.octree import soma_octree
from mongodb_store.message_store import MessageStoreProxy

"""
A tool that can excute query manipulation manully.
"""

if __name__ == '__main__':
    # Setup Connection
    rospy.init_node("query_example")
    msg_store = MessageStoreProxy(database='message_store', collection='ws_observations')

    # Query Octree (messge_store api)
    oct_msg_list = msg_store.query('octomap_msgs/Octomap')

    # Deserialize message
    # TODO: can't read the data directly form stream
    # Load octree from binary octree and add some other info for the py-octomap read() API
    # minimum requirement is oct_head, oct_data, oct_res, oct_data_keyword and oct_data_real_data

    #for i in range(0,len(oct_msg_list)):
    for i in range(0, 2):
        i = 0
        oct_template = '# Octomap OcTree binary file\n'
        oct_template += 'id ' + oct_msg_list[i][0].id + '\n'
        oct_template += 'res ' + str(oct_msg_list[i][0].resolution) + '\n'
        oct_template += 'data\n'
        oct_template += oct_msg_list[i][0].data

        # Load octree form query result
        octr = soma_octree(oct_template)
        print type(octr)
        print oct.octree.size()
        #oct.octree.readBinary(oct_template)
        oct.get_bbx_info(oct.octree)
        """
        if oct.point_in_observation(oct.octree,10,9,8):
            print 'point exist in observation ' + str(i)
        else:
            print 'point not in observation ' + str(i)
        """

    #octree = octomap.OcTree(oct_msg_list[i][0].resolution)
    #octree.readBinary(oct_template)



    #check if point is covered

    #return result
