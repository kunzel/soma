Header header
string id 
string map_name
string map_unique_id 
string config
string mesh
string type
string logtime
string waypoint
string timestep
int8 logday
int8 loghour
int8 logminute
geometry_msgs/Pose pose
geometry_msgs/Pose sweepCenter
sensor_msgs/PointCloud2 cloud
geometry_msgs/PoseArray posearray
# geotype: Point,Polygon,etc
string geotype
# the poses in geospatial format lat,long size= posearray+1 for forming the polygon
geometry_msgs/PoseArray geoposearray
#the loghour+logminute in terms of total minutes to simplify queries
int32 logtimeminutes
