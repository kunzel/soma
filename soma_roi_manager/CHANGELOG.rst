^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package soma_roi_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2016-06-20)
------------------
* Marc hanheide patch for release (`#44 <https://github.com/strands-project/soma/issues/44>`_)
  * updated changelogs
  * 1.0.1
  * updated changelogs
  * 1.0.2
  * added octomap_msgs dep
  * added libqt5-core
  * removed invalid install target
  * added interactive_markers dep
  * fix dev for QT5
* Contributors: Marc Hanheide

1.0.2 (2016-06-07)
------------------

1.0.1 (2016-06-06)
------------------

1.0.0 (2016-06-06)
------------------
* Updated package xml files.
* Added missing import
* Big restucture for release
* Updated structure for release.
* Fixed the hanging of query_manager during shutdown. Refined code for soma_roi
* The corruption of regions after saving and loading has been fixed
* Query results are returned as srv response. ROI drawing color has been set
* SOMA2  to SOMA name changes
* Updated documentation
* Latest soma2 is merged into soma fork named as soma
* Contributors: Nick Hawes, hkaraoguz

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
* Add get_rois function to query class.
* add cs_1 config file for soma_roi
* g4s demo
* fixing the trajectory_importer.py
* sort menu items
* convert xy-coords to longitude/latitude
* use geopatial store for ROIs
* replaced ros_datacentre with mongodb_store
* added labels for ROIs
* removed debug printout
* fixed problem with polygon
* visualize roi using a line strip marker
* added manager for ROIs
* Contributors: Chris Burbridge, Ferdian Jovan, Lars Kunze, Marc Hanheide, Nick Hawes
