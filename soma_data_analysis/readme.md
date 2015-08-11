SOMA data analysis 
------------------
Generate the data:

    ```
    $ roslaunch semantic_map_launcher semantic_map.launch 
    ```

    ```
    $ roslaunch semantic_segmentation semantic_segmentation_plus.launch 
    ```
    ```
    $ rosrun soma_data_analysis my_pcl_segmentation.py 
    ```
Run the SOMA object analysis:

    ```
    $ rosrun soma_data_analysis object_clustering.py 
    ```
input:

1.for request waypoint, input one waypoint in the form: "WayPoint42"

2.for operation, input one of the following operations:
  * "box"    :  generate and display bounding boxes given one label and one instance
  * "cloud"  :  display the point cloud of one label and one instance
  * "single" :  compute the distribution of the movement of a single object given a list of instances, one label and one specific object and display the region where this object is movable.
  * "tv_spatial" : compute the possibility of a tv being on a table given a list of instances.
  * "chair_spatial" : compute the possibility of a chair being near a table given a list of instances.

3.for request instances, input instances corresponding to the operation in the form: [0] or [0,1,2,5]

4.for request label, input the number representing the label: (range from 0 to 10)

   * label_type=["prop","wall","cabinet","ceiling","chair/sofa", "window", "floor","monitor/tv","person","shelf", "table"]

5.for specific object(especially for mission 'single') : input a number refering to an object in the first instance of the querying list.

Run the SOMA region analysis:

    ```
    $ rosrun soma_data_analysis region_identifying.py 
    ```
input:

1. for request waypoint & instance, input one waypoint and one instance in the form: ["WayPoint15",[0]]

2. for operation, input one of the following operations: 
  * "outter"    :  generate and display the outter region given one waypoint and instance.
  * "inner"     :  generate and display the region that robot can move freely given one waypoint and    instance.



