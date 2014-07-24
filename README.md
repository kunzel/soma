soma
====

A toolbox for managing Semantic Object Maps (SOMA). Currently there are two modeling tools: 

- the SOMA object manager 
- the SOMA ROI manager (ROI = regions of interest)
 
The SOMA manager for objects visualizes objects in RVIZ where the user can add, delete, move, and rotate them interactively. The SOMA manager for ROIs visualizes regions in RVIZ where the user can add, delete, and modify them. Both the object and the ROI configurations are immediately stored and updated in the ROS datacentre (MongoDB).  


Prerequisites
-------------

- MongoDB
- ros_datacentre
- ROS's navigation stack 


Getting started (general steps)
-------------------------------
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
    

4. Start RVIZ, add a Map display type and subsribe to the `/map` topic:

  ```
  $ rosrun rviz rviz
  ```

SOMA object manager
-------------------

5. Run the SOMA object manager:

    ```
    $ rosrun soma_manager soma.py <map> <config>
    ```
where `map` denotes the name of the 2D map (Step 3) and `config` denotes an object configuration within this map. By default, the configuration file `soma_objects/config/default.json` is used to initialize the list of available object types. Alternatively, the following command can be used to use a different configuration file:

    ```
    $ rosrun soma_manager soma.py -t /path/to/config/file <map> <config>
    ```
6. In RVIZ, add an InteractiveMarker display type, and subsribe to the `/soma/update` topic:
7. Add, delete, move, and rotate objects in RVIZ using the interactive marker and the context menu (right-mouse-click)

![marker](https://raw.githubusercontent.com/kunzel/soma/master/doc/images/soma_manager.png)

SOMA ROI manager
----------------

5. Run the SOMA ROI manager:

    ```
    $ rosrun soma_roi_manager soma_roi.py <map> <config>
    ```
where `map` denotes the name of the 2D map (Step 3) and `config` denotes an object configuration within this map. By default, the configuration file `soma_roi_manager/config/default.json` is used to initialize the list of available ROI types. Alternatively, the following command can be used to use a different configuration file:

    ```
    $ rosrun soma_roi_manager soma_roi.py -t /path/to/config/file <map> <config>
    ```
6. In RVIZ, add an InteractiveMarker display type, and subsribe to the `/soma_roi/update` topic:
7. Add, delete, modify ROIs in RVIZ using the interactive marker and the context menu (right-mouse-click)


![marker](https://raw.githubusercontent.com/kunzel/soma/master/doc/images/soma_roi.png)
