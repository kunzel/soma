#!/usr/bin/env python
import roslib
roslib.load_manifest("soma_geospatial_store")
import rospy
import pymongo
import math

class TwoProxies(object):
    """Allows you to use spatial queries on geo_store, and returns message_store document."""
    def __init__(self, geo_store, message_store, map, config):
        self.gs = geo_store
        self.msg = message_store
        self.map = map
        self.config = config

    def object_xyz(self, res):
        objects={}
        for i in res:
            key = i['type'] +'_'+ i['soma_id']
            objects[key] = self.msg.obj_coords(i['soma_id'], self.map, self.config)
        return objects


    def trajectory_roi(self, uuid, poses): 
        return self.gs.trajectory_roi(uuid, poses, self.map, self.config)


    def roi_objects(self, roi):
        geom = self.gs.geom_of_roi(str(roi), self.map, self.config)  #roi geometry
        res = self.gs.objs_within_roi(geom, self.map, self.config)   #objs in roi
        if res == None:
            return None
        return self.object_xyz(res)




class GeoSpatialStoreProxy():
        
    def __init__(self, db, collection):
        host = rospy.get_param('mongodb_host', 'localhost')
        port = rospy.get_param('mongodb_port', '62345')
        self._client = pymongo.MongoClient(host,port)
        self._db = db
        self._collection = collection

        # create indexes
        self._client[self._db][self._collection].ensure_index([("loc", pymongo.GEOSPHERE)])

        # self._client[self._db][self._collection].ensure_index([("uuid", 1)], 
        #                                                       unique= True,
        #                                                       sparse=True)

        # self._client[self._db][self._collection].ensure_index([("soma_id", 1),("soma_map",1),("soma_config",1)],
        #                                                       unique=True,
        #                                                       sparse=True)

        # self._client[self._db][self._collection].ensure_index([("soma_roi_id",1),("soma_map",1),("soma_config",1)], 
        #                                                       unique= True,
        #                                                       sparse=True)

                
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

    def find_projection(self, query_json, projection):
        return self._client[self._db][self._collection].find(query_json, projection)


    def coords_to_lnglat(self, x, y):
        earth_radius = 6371000.0 # in meters
        lng = 90 - math.degrees(math.acos(float(x) / earth_radius))
        lat = 90 - math.degrees(math.acos(float(y) / earth_radius))        
        return [lng , lat]

    def area(self, p):
        return 0.5 * abs(sum(x0*y1 - x1*y0 for ((x0, y0), (x1, y1)) in self.segments(p)))

    def segments(self, p):
        return zip(p, p[1:] + [p[0]])
     
    # helper functions
    def obj_ids(self, soma_map, soma_config):
        query =  {   "soma_id": {"$exists": "true"},
                     "soma_map":  soma_map ,
                    "soma_config": soma_config
        }
        res = self.find_projection(query, {"soma_id": 1})
        ret = []
        for ent in res:
            ret.append(ent["soma_id"])
        return ret


    def trajectory_roi(self, uuid, poses, soma_map, soma_config): 
        """Returns region of the trajectory"""
        geom = []
        for p in poses:
            geom.append(self.coords_to_lnglat(p[0], p[1]))

        query = {"soma_map":  soma_map,
                 "soma_config": soma_config,
                 "soma_roi_id": {"$exists": "true"},
                 "loc": {"$geoIntersects": {
                         "$geometry": {
                             "type" : "LineString", 
                             "coordinates" : geom}}}
                }
        
        res = self.find_projection(query, {"soma_roi_id": 1})
        try:
            ret = res[0]['soma_roi_id']
        except:
            print "ERROR: no ROI found"
            ret = None
        return ret  


    def roi_ids(self, soma_map, soma_config):
        query =  {  "soma_roi_id": {"$exists": "true"},
                    "soma_map":  soma_map ,
                    "soma_config": soma_config
                }
        res = self.find_projection(query, {"soma_roi_id": 1})
        ret = []
        for ent in res:
            ret.append(ent["soma_roi_id"])
        return ret

    def type_of_obj(self, soma_id, soma_map, soma_config):
        query =  { "soma_id": soma_id,
                   "soma_map":  soma_map ,
                   "soma_config": soma_config
        }
        res = self.find_projection(query, {"type": 1})
        if res.count() == 0:
            return None
        return res[0]['type']

    def type_of_roi(self, soma_roi_id, soma_map, soma_config):
        query =  { "soma_roi_id": soma_roi_id,
                   "soma_map":  soma_map ,
                   "soma_config": soma_config
        }
        res = self.find_projection(query, {"type": 1})
        if res.count() == 0:
            return None
        return res[0]['type']

        
    def geom_of_obj(self, soma_id, soma_map, soma_config):
        query =  { "soma_id": soma_id,
                   "soma_map":  soma_map ,
                   "soma_config": soma_config
        }
        res = self.find_projection(query, {"loc": 1})
        if res.count() == 0:
            return None
        return res[0]['loc']

        
    def geom_of_roi(self, roi_id, soma_map, soma_config):
        query =  { "soma_roi_id": roi_id,
                   "soma_map":  soma_map ,
                   "soma_config": soma_config
        }
        res = self.find_projection(query, {"loc": 1})
        if res.count() == 0:
            return None
        return res[0]['loc']
        
    def geom_of_trajectory(self, uuid):
        query =  { "uuid": uuid}
        res = self.find_projection(query, {"loc": 1})
        if res.count() == 0:
            return None
        return res[0]['loc']


    def objs_within_roi(self, roi, soma_map, soma_config):
        """Returns all the objects within a region of interest"""

        query = {  "soma_map":  soma_map ,
                   "soma_config": soma_config,
                   "soma_id": {"$exists": "true"},
                   "loc": {"$geoIntersects": {"$geometry": roi}} 
                }

        res = self.find(query)
        if res.count() == 0:
            return None
        return res

    def trajectories_within_roi(self, roi, soma_map, soma_config):
        """Returns all the trajectories within a region of interest"""

        query = {  "soma_map":  soma_map ,
                   "soma_config": soma_config,
                   "soma_id": {"$exists": "true"},
                   "loc": {"$geoIntersects": {"$geometry": roi}} 
                }

        res = self.find(query)
        if res.count() == 0:
            return None
        return res


    def obj_coords(self, soma_id, soma_map, soma_config):
        """Returns the map coordinates of a soma_id object"""
        query = {  "map":  soma_map,
                   "config": soma_config,
                   "id": soma_id
                } 

        res = self.find_projection(query, {"pose": 1})

        if res.count() == 0:
            return None
        return res[0]['pose']['position']['x'], res[0]['pose']['position']['y'], \
            res[0]['pose']['position']['z']


    def observed_roi(self, view_tri, soma_map, soma_config):
        """Returns list of regions the robot can see"""
        query = {  "soma_map":  soma_map ,
                   "soma_config": soma_config,
                   "soma_roi_id": {"$exists": "true"},
                   "loc": {"$geoIntersects": {
                         "$geometry": { 
                             "type" : "Polygon", 
                             "coordinates" : [view_tri]}}}
                } 
        res = self.find_projection(query, {"soma_roi_id" : 1})
        return res


    def area_of_roi(self, roi_id, soma_map, soma_config):
        """Returns the area of ROI in map coords"""
        query =  { "roi_id": roi_id,
                   "map":  soma_map ,
                   "config": soma_config
        }
        res = self.find_projection(query, {"pose": 1})
        coords = []
        for i in res:
            coords.append((i['pose']['position']['x'], i['pose']['position']['y']))

        if res.count() == 0:
            return None
        return self.area(coords)


