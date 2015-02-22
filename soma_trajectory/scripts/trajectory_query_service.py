#!/usr/bin/env python
import roslib; roslib.load_manifest("soma_trajectory")
import rospy
import argparse
import math
import json

from human_trajectory.msg import Trajectory
from human_trajectory.msg import Trajectories

from soma_geospatial_store.geospatial_store import GeoSpatialStoreProxy 
from mongodb_store.message_store import MessageStoreProxy

from soma_trajectory.srv import TrajectoryQuery, TrajectoryQueryResponse
from soma_trajectory.visualizer import TrajectoryVisualizer

class TrajectoryQueryService():

    def __init__(self):
        self.gs = GeoSpatialStoreProxy('geospatial_store','soma')
        self.ms = MessageStoreProxy(collection="people_trajectories")

        # setting up the service
        self.ser = rospy.Service('/trajectory_query', TrajectoryQuery, self.service_cb)
        self.topic = 'trajectory_query'
        self.vis = TrajectoryVisualizer(self.topic)
        
    def retrieve_msg(self, uuid):
        res = self.ms.query(Trajectory._type, message_query={"uuid": uuid})
        if len(res) < 1:
            rospy.logerr("Trajectory not found: %s" % uuid)
            return None
        elif len(res) > 1:
            rospy.logerr("Multiple trajectories found: %s" % uuid)
            return None

        t = res[0][0]
        return t
        
    def service_cb(self, req):
        rospy.loginfo("Request received: %s" % req)
        if req.visualize:
            self.vis.clear()
        
        res = TrajectoryQueryResponse()
        res.trajectories = Trajectories()
        try:
            json_query = json.loads(req.query)
            trajectories = self.gs.find(json_query)
        except:
            rospy.logerr("Invalid json => re-check syntax")
            res.error = True
            return res

        count = 0
        for t in trajectories:
            if t.has_key('uuid'):
                count += 1
                #rospy.loginfo("retrieve msg for uuid: %s" % t['uuid'])
                # otherwise result is not a trajectory => ignore
                msg = self.retrieve_msg(t['uuid'])
                if msg:
                    res.trajectories.trajectories.append(msg)

        rospy.loginfo("Query result: %s trajectories" % count)

        if req.visualize:
            rospy.loginfo("Visualize result on topic: %s" % self.topic)
            self.vis.visualize_trajectories(res.trajectories)
        rospy.loginfo("Response returned")
        res.error = False
        return res

    def main(self):
        rospy.spin()

if __name__=="__main__":
    rospy.init_node("trajectory_query_service")    
    rospy.loginfo("Running trajectory_query_service")
    tqs = TrajectoryQueryService()
    tqs.main()
      

