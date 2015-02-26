#!/usr/bin/env python
import roslib; roslib.load_manifest("soma_trajectory")
import rospy
import json

from human_trajectory.msg import Trajectory
from human_trajectory.msg import Trajectories
from soma_trajectory.srv import TrajectoryQuery, TrajectoryQueryRequest, TrajectoryQueryResponse

from soma_geospatial_store.geospatial_store import GeoSpatialStoreProxy

class QueryClient():

    def __init__(self):
        service_name = 'trajectory_query'
        rospy.wait_for_service(service_name)
        self.ser = rospy.ServiceProxy(service_name, TrajectoryQuery)

    def query(self, query, vis = False):
        try:
            req = TrajectoryQueryRequest()
            req.query = query
            req.visualize = vis
            res = self.ser(req)
            return res
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)


if __name__=="__main__":
    rospy.init_node("query_examples")    
    rospy.loginfo("Running query examples")

    client = QueryClient()

    gs = GeoSpatialStoreProxy('geospatial_store','soma')
    soma_map = 'library'
    soma_config = 'test'
    
    for obj in gs.obj_ids(soma_map, soma_config):
        print 'OBJ', obj,  gs.type_of_obj(obj, soma_map, soma_config)
        geom = gs.geom_of_obj(obj, soma_map, soma_config)
        print 'OBJ', obj, geom


    for roi in gs.roi_ids(soma_map, soma_config):
        print 'ROI',gs.type_of_roi(roi, soma_map, soma_config)
        geom = gs.geom_of_roi(roi, soma_map, soma_config)
        print 'ROI', roi, geom

    geom = gs.geom_of_trajectory('328e2f8c-6147-5525-93c4-1b281887623b')
    print 'TRAJECTORY', geom
        
    # intersects with other shape [geometry is one of point, polygon, linestring]
    # within region of interest (ROI) [geometry is a polygon]
    query = '{"loc": { "$geoIntersects": { "$geometry":' + json.dumps(geom) + '}}}'

    rospy.loginfo("Query: %s" % query )
    res = client.query(query, True)
    rospy.loginfo("Result: %s trajectories" % len(res.trajectories.trajectories))


    

    
