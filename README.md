SOMA2
====

An enhanced version of Semantic Object Map (SOMA) package. SOMAs can include objects, regions of interest (ROI), and trajectories.


Prerequisites
-------------

- MongoDB (>=2.6)
- mongodb_store - use the source version from github
- ROS's navigation stack (only map server)
- Qt5 (sudo apt-get install qtbase5-dev)


Getting started (general steps)
-------------------------------
1. Start the ros core:

    ```
   $ roscore
    ```
2. Launch the ROS datacentre:

    ```
    $ roslaunch mongodb_store mongodb_store.launch db_path:=<path_to_db>
    ```

SOMA map manager
----------------
3. SOMA2 is currently based on the assumption that all the detected objects are in a 2D global map frame. So **it is required to run the soma map manager** for using SOMA2. This node is used for storing, reading and publishing 2D map:
```
$ rosrun soma_map_manager soma_map.py
```
If there are any stored 2D occupancy maps in the datacenter, then map manager will let you choose the map to be published. If not, it will wait for map_server. Run the map_server with a 2D map:
  ```
  $ rosrun map_server map_server <map.yaml>
  ```
where `map.yaml` specifies the map you want to load. After running the `map_server`, you should save the published map using the `SOMA map manager`.

4. If you want to check the published map, start RVIZ, add a Map display type and subscribe to the `soma2/map` topic:

  ```
  $ rosrun rviz rviz
  ```

SOMA ROI manager
----------------

5. If you want to create SOMA ROIs, run the SOMA ROI manager:

    ```
    $ rosrun soma_roi_manager soma_roi.py <config>
    ```
where `config` denotes an object configuration. By default, the configuration file `soma_roi_manager/config/default.json` is used to initialize the list of available ROI types. Alternatively, the following command can be used to use a different configuration file:

    ```
    $ rosrun soma_roi_manager soma_roi.py -t /path/to/config/file <config>
    ```
2D `map` information will be gathered from `soma2/map_info` service of `SOMA map manager`.
6. In RVIZ, add an InteractiveMarker display type, and subscribe to the `/soma2_roi/update` topic:
7. Add, delete, modify ROIs in RVIZ using the interactive marker and the context menu (right-mouse-click)


![marker](https://raw.githubusercontent.com/kunzel/soma/master/doc/images/soma_roi.png)

ROS Services
--------
The other nodes can communicate with SOMA2 using the SOMA2 service calls. In order to use these services, one should run the soma data manager:
## SOMA data manager
1. Run the soma data manager:
```
$ rosrun soma_manager data_manager.py <db_name> <collection_name>
```
The parameters `db_name` and `collection_name` are optional which can be used to define the database and collection name for data storage.
### Data insertion
One or multiple SOMA2 objects can be inserted using the SOMA2 service call `/soma2/insert_objects`. The unique mongodb ids and a boolean value are returned. The boolean return value determines whether the request was successfully completed or not.
### Data deletion
One or multiple SOMA2 objects can be deleted using the SOMA2 service call `/soma2/delete_objects`. The SOMA2 object ids are used for deletion. The boolean return value determines whether the request was successfully completed or not.
### Data update
A SOMA2 object can be updated using the SOMA2 service call `/soma2/update_object`. The boolean return value determines whether the request was successfully completed or not.
### Data query
SOMA2 objects and ROIs could be queried using SOMA2 service call `/soma2/query_db`. The query request should be filled according to the spatio-temporal constraints. The results are returned based on the query type and constraints.
