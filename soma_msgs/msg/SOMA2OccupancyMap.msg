# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#Map data
nav_msgs/OccupancyGrid map

# The name of the map stored
string mapname

#rle_value
int32[] rle_values

#rle_counts
int32[] rle_counts
