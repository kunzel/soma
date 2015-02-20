#!/usr/bin/env python
import roslib
roslib.load_manifest("soma_geospatial_store")
import rospy
import pymongo


class GeoSpatialStoreProxy():
        
    def __init__(self, db, collection):
        host = rospy.get_param('mongodb_host', 'localhost')
        port = rospy.get_param('mongodb_port', '62345')
        self._client = pymongo.MongoClient(host,port)
        self._db = db
        self._collection = collection
        self._client[self._db][self._collection].ensure_index([("loc", pymongo.GEOSPHERE)])
        
    def insert(self, geo_json):
        return self._client[self._db][self._collection].insert(geo_json)

    def update(self, _id, geo_json):
        self.remove(_id)
        return self.insert(geo_json)

    def remove(self, _id):
        return self._client[self._db][self._collection].remove(_id)

    def find_one(self, query_json):
        return self._client[self._db][self._collection].find_one(query_json)
        
    def find(self, query_json):
        return self._client[self._db][self._collection].find(query_json)

    def find(self, query_json, projection):
        return self._client[self._db][self._collection].find(query_json, projection)


class GeoSpatialUtils():
    
    def line_string_from_pose_array(pose_arr):
        pass

    def polygon_from_pose_array(pose_arr):
        pass

    def point_from_pose(pose):
        pass
