SOMA data analysis 
------------------
Generate the data:

    ```
    $ roslaunch semantic_map_launcher semantic_map.launch 
    ```

    ```
    $ roslaunch semantic_segmentation semantic_segmentation_integrate.launch 
    ```
    ```
    $ rosrun soma_data_analysis my_pcl_segmentation.py 
    ```
Run the SOMA object analysis:

    ```
    $ rosrun soma_data_analysis object_clustering.py 
    ```
input:

1. for request waypoint, input one waypoint in the form: "WayPoint42"

2. for operation, input one of the following operations:
  * "box"    :  generate and display bounding boxes given one label and one instance
  * "cloud"  :  display the point cloud of one label and one instance
  * "single" :  compute the distribution of the movement of a single object given a list of instances, one label and one specific object and display the region where this object is movable.
  * "tv_spatial" : compute the possibility of a tv being on a table given a list of instances.
  * "chair_spatial" : compute the possibility of a chair being near a table given a list of instances.
  * "predict" : predict the scene ( positions of table, chair and monitor) of one waypoint given a list of instances.

3. for request instances, input instances corresponding to the operation in the form: [0] or [0,1,2,5]

4. for request label, input the number representing the label: (range from 0 to 10)

   * label_type=["prop","wall","cabinet","ceiling","chair/sofa", "window", "floor","monitor/tv","person","shelf", "table"]

5. for specific object(especially for mission 'single') : input a number refering to an object in the first instance of the querying list.

output:

1. box&cloud function:(waypoint15,instance0,label10[table])

(yellow part is the acutal cloud)

![marker](https://raw.githubusercontent.com/duanby/soma/data_analysis/soma_data_analysis/images/box&cloud.png)

2. single&box function:(waypoint15,instance0,label10[table],object0)

(red region represent the region the object has been to)

![marker](https://raw.githubusercontent.com/duanby/soma/data_analysis/soma_data_analysis/images/single.png)

3. predict function:(waypoint15,instance[0,1,2,3,4])

(red boxes represent table; green boxes represent chair/sofa; other boxes represent tv/monitor)

![marker](https://raw.githubusercontent.com/duanby/soma/data_analysis/soma_data_analysis/images/predict.png)

Run the SOMA region analysis:

    ```
    $ rosrun soma_data_analysis region_identifying.py 
    ```
input:

1. for request waypoint & instance, input one waypoint and one instance in the form: ["WayPoint15",[0]]

2. for operation, input one of the following operations: 
  * "outter"    :  generate and display the outter region given one waypoint and instance.
  * "inner"     :  generate and display the region that robot can move freely given one waypoint and    instance.

output:

1. box&cloud function:(waypoint15,instance0)

(green dots represent the convex of outter region)

(reg region represents the region that robot can move freely)

![marker](https://raw.githubusercontent.com/duanby/soma/data_analysis/soma_data_analysis/images/roi.png)

Run the SOMA room type analysis:

    ```
    $ rosrun soma_data_analysis room_type_classifying.py 
    ```
input:

1. for operation, input one of the following operations: 
  * "test"    :  predict a list of instances from a list of waypoints and display the accuracy
  * "predict" :  predict the room type of a list of query instances and its actual label

2. for request waypoints(especially for "predict" function), input a list of waypoints in the form: ["WayPoint15","WayPoint6"]

3. for request instances(especially for "predict" function), input several list of instances corresponding to the waypoints: [0,1] [1,2,3,4,5]

output:
from 0 to 4, each number of the output is the index of the types of room:
[Kitchen, OpenPlan, Office, GroupOffice,MeetingRoom]

    ```
    $ rosrun soma_data_analysis room_type_clustering.py 
    ```
input:

1. for operation, input one of the following operations: 
  * "whole"  :  cluster among all the instances of all the waypoints
  * "aver"   :  cluster among all the waypoints (its feature is the average of feature inside the waypoint)
  * "expand" :  firstly, cluster inside every waypoint, then treat every centroid as an independent waypoint.      secondly, cluster these new "waypoints" using the method similiar to "aver".

output:

1. for "whole" operation: output is display in the form of [waypoint, instance, index of cluster it belongs to]

2. for other operation: output is display in the form of [waypoint, index of cluster it belongs to]

