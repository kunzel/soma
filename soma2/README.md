soma2
====

An enhanced version of Semantic Object Map (SOMA) package. SOMAs can include objects, regions of interest (ROI), and trajectories.


Prerequisites
-------------

- MongoDB (>=2.6)
- mongodb_store - use this [fork](https://github.com/hkaraoguz/mongodb_store)
- ROS's navigation stack (only map server)


Getting started (general steps)
-------------------------------
1. Start the ros core:

    ```
   $ roscore
    ```
2. Launch the ROS datacentre:

    ```
    $ roslaunch mongodb_store datacentre.launch
    ```

SOMA2 map manager
----------------
3. Run the soma2 map manager for storing, reading and publishing 2D map:
```
$ rosrun soma2_map_manager soma2_map.py
```
If there are any stored 2D occupancy maps in the datacentre then map manager will let you choose the map to be published. If not, it will wait for map_server. Run the map_server with a 2D map:
  ```
  $ rosrun map_server map_server <map.yaml>
  ```
where `map.yaml` specifies the map you want to load. After running the `map_server`, you should save the published map using the `soma2 map manager`.

4. Start RVIZ, add a Map display type and subscribe to the `soma2/map` topic:

  ```
  $ rosrun rviz rviz
  ```

SOMA2 ROI manager
----------------

5. Run the SOMA2 ROI manager:

    ```
    $ rosrun soma2_roi_manager soma_roi.py <config>
    ```
where `config` denotes an object configuration. By default, the configuration file `soma_roi_manager/config/default.json` is used to initialize the list of available ROI types. Alternatively, the following command can be used to use a different configuration file:

    ```
    $ rosrun soma2_roi_manager soma_roi.py -t /path/to/config/file <config>
    ```
2D `map` information will be gathered from `soma2/map_info` service of `soma2_map_manager`.
6. In RVIZ, add an InteractiveMarker display type, and subsribe to the `/soma2_roi/update` topic:
7. Add, delete, modify ROIs in RVIZ using the interactive marker and the context menu (right-mouse-click)


![marker](https://raw.githubusercontent.com/kunzel/soma/master/doc/images/soma_roi.png)
