#!/usr/bin/env python
import roslib; roslib.load_manifest("soma_trajectory")
import rospy

from human_trajectory.msg import Trajectory
from human_trajectory.msg import Trajectories
from soma_trajectory.srv import TrajectoryQuery, TrajectoryQueryRequest, TrajectoryQueryResponse

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

    ## Get all trajectories
    # query = '{}'
    # rospy.loginfo("Query: %s" % query )
    # res = client.query(query, True)
    # rospy.loginfo("Result: %s trajectories" % len(res.trajectories.trajectories))

    #raw_input("Press enter to continue")
    
    # Malformed queries lead to an error => always check error flag in result!
    query = '{'
    rospy.loginfo("Query: %s" % query)
    res = client.query(query, True)
    if res.error:
        rospy.logerr("Result: error: %s (Invalid query: %s)" % (res.error, query))

    raw_input("Press enter to continue")
        
    # TEMPORAL QUERIES

    # Available attributes (example):
    # "start" : 1416840575.527016,
    # "end" : 1416840578.927002,
    # "start_hour" : 14,
    # "end_hour" : 14,
    # "start_minute" : 49,
    # "end_minute" : 49,
    # "start_weekday" : 0,
    # "end_weekday" : 0
    
    # Get all trajectories that start between 14-15 o'clock 
    query = '{"start_hour": {"$gt": 13, "$lt": 15}}'
    rospy.loginfo("Query: %s" % query )
    res = client.query(query, True)
    rospy.loginfo("Result: %s trajectories" % len(res.trajectories.trajectories))

    raw_input("Press enter to continue")
    
    # Get all trajectories that start between 14-15 o'clock and weekday is 0  
    query = '{"start_hour": {"$gt": 13, "$lt": 15}, "start_weekday": 0}'
    rospy.loginfo("Query: %s" % query )
    res = client.query(query, True)
    rospy.loginfo("Result: %s trajectories" % len(res.trajectories.trajectories))

    raw_input("Press enter to continue")
    
    # Get all trajectories that start between 14-15 o'clock and end between 15-16 o'clock
    query = '{"start_hour": {"$gt": 13, "$lt": 15}, "end_hour": {"$gt": 14, "$lt": 16}}'
    rospy.loginfo("Query: %s" % query )
    res = client.query(query, True)
    rospy.loginfo("Result: %s trajectories" % len(res.trajectories.trajectories))

    raw_input("Press enter to continue")
    
    # # GEO SPATIAL QUERIES

    # near object [geometry is a point, maxDistance in meters]
    query = '{"loc": {"$nearSphere": { "$geometry":  { "type" : "Point", "coordinates" : [ -0.0002281133006505343, -4.632269674686995e-05 ] }, "$maxDistance": 1}}}'
    rospy.loginfo("Query: %s" % query )
    res = client.query(query, True)
    rospy.loginfo("Result: %s trajectories" % len(res.trajectories.trajectories))

    raw_input("Press enter to continue")

    # within region of interest (ROI) [geometry is a polygon]
    query ='''{"loc": { "$geoWithin": { "$geometry":
    {
        "type" : "Polygon",
        "coordinates" : [ 
            [ 
                [ 
                    -0.0002246355582968818, 
                    -2.519034444503632e-05
                ], 
                [ 
                    -0.0002241486476179944, 
                    -7.42736662147081e-05
                ], 
                [ 
                    -0.000258645873657315, 
                    -7.284014769481928e-05
                ], 
                [ 
                    -0.0002555339747090102, 
                    -2.521782172948406e-05
                ], 
                [ 
                    -0.0002246355582968818, 
                    -2.519034444503632e-05
                ]
            ]
        ]
    }}}}'''
    rospy.loginfo("Query: %s" % query )
    res = client.query(query, True)
    rospy.loginfo("Result: %s trajectories" % len(res.trajectories.trajectories))

    raw_input("Press enter to continue")

    # intersects with other shape [geometry is one of point, polygon, linestring]
    # within region of interest (ROI) [geometry is a polygon]
    query ='''{"loc": { "$geoIntersects": { "$geometry": {     
        "type" : "Polygon",
        "coordinates" : [ 
            [
                           [ 
                    -0.0002246355582968818, 
                    -2.519034444503632e-05
                ], 
                [ 
                    -0.0002241486476179944, 
                    -7.42736662147081e-05
                ], 
                [ 
                    -0.000258645873657315, 
                    -7.284014769481928e-05
                ], 
                [ 
                    -0.0002555339747090102, 
                    -2.521782172948406e-05
                ], 
                [ 
                    -0.0002246355582968818, 
                    -2.519034444503632e-05
                ]
            ]
        ]                                                    
    }}}}'''
    rospy.loginfo("Query: %s" % query )
    res = client.query(query, True)
    rospy.loginfo("Result: %s trajectories" % len(res.trajectories.trajectories))


    

    
