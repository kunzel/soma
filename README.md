soma
====

A toolbox for managing Semantic Object Maps (SOMA).


Prerequisites
-------------

- MongoDB
- ros_datacentre
- ROS's navigation stack 


Getting started
---------------
1. Start the ros core:

    ```
   $ roscore
    ```
2. Launch the ROS datacentre:

    ```
    $ roslaunch ros_datacentre datacentre.launch
    ```
3. Run the map_server with a 2D map:
  ```
  $ rosrun map_server map_server <map.yaml>
  ```
where `map.yaml` specifies the map you want to load.
    
4. Run the SOMA object manager:

    ```
    $ rosrun soma_manager soma.py <map> <config>
    ```
where `map` denotes the name of the 2D map (Step 3) and `config` denotes an object configuration within this map. By default, the configuration file `soma_objects/config/default.json` is used to initialize the list of available object types. Alternatively, the following command can be used to use a different configuration file:

    ```
    rosrun soma_manager soma.py -t /path/to/config/file <map> <config>
    ```
5. Finally, start RVIZ, add an InteractiveMarker display type, and subsribe to the `/soma/update` topic.

  ```
  $ rosrun rviz rviz
  ```











