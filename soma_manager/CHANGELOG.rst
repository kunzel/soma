^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package soma_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2016-06-06)
------------------
* Updated package xml files.
* Big restucture for release
* Updated structure for release.
* Fixes for building on OSX and restructure for release.
* Fixed the time based query problem by changing the field type of lowerdate and upperdate to uint64
* fix for header dependency
* Try to solve query manager lock down while ros::shutdown. Moved to launch folder under soma_manager
* Fixed the hanging of query_manager during shutdown. Refined code for soma_roi
* Fixed issues with query manager crashing when empty fields or no data is present
* Fixed issue with empty typeids or objectsids. Updated launch file.
* Corrected msg dependencies
* Query results are returned as srv response. ROI drawing color has been set
* Listening map service is added
* Query service has been added
* Remove unnecesary folder
* Updated documentation
* Latest soma2 is merged into soma fork named as soma
* Contributors: Nick Hawes, Nils Bore, hkaraoguz

0.0.2 (2016-06-06)
------------------
* updated changelogs
* Contributors: Jenkins

0.0.1 (2016-02-03)
------------------
* added changelogs
* Fixed args so they work in a launch file.
* fix frame problem
* increase z offset for marker
* sort menu items
* convert xy-coords to longitude/latitude
* add dependency
* geospatial store for semantic object maps
* replaced ros_datacentre with mongodb_store
* renamed ros node
* fixed node name
* add new objects always on the ground plane
* added possibility of modelling object in 3D
* fixed indention
* read available object types from config file
* enable/disable interactive markers
* initial commit
* Contributors: Lars Kunze, Marc Hanheide, Nick Hawes
