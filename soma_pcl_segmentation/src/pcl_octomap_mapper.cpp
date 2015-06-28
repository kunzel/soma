//Ros includes
#include "ros/ros.h"

//STL includes
#include <vector>

//Service includes
#include "soma_pcl_segmentation/GetPCLOctomapMapping.h"

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include "geometry_msgs/Point32.h"
using namespace std;
using namespace octomap;
#define ANGLE_MAX_DIFF (M_PI / 8)  

OcTree* extract_supporting_planes(OcTree* tree)
{
  OcTree* sp_tree = new OcTree(tree->getResolution());  

  int free = 0;
  int occupied = 0;
  int supported = 0;
  
  ROS_INFO("Extracting supporting planes from octomap");
  
  for(OcTree::leaf_iterator it = tree->begin_leafs(),
        end=tree->end_leafs(); it!= end; ++it)
    {
      if (tree->isNodeOccupied(*it))
        {
          occupied++;
          std::vector<point3d> normals;
          
          point3d p3d = it.getCoordinate();

          bool got_normals = tree->getNormals(p3d ,normals, true); 
          std::vector<point3d>::iterator normal_iter;
          
          point3d avg_normal (0.0, 0.0, 0.0);
          for(std::vector<point3d>::iterator normal_iter = normals.begin(), 
                end = normals.end(); normal_iter!= end; ++normal_iter)
            {
              avg_normal+= (*normal_iter);
            }
          if (normals.size() > 0) 
            {
              supported++;
              // cout << "#Normals: " << normals.size() << endl;

              avg_normal/= normals.size();       
              
              point3d z_axis ( 0.0, 0.0, 1.0);
              double angle = avg_normal.angleTo(z_axis);

              point3d coord = it.getCoordinate();

              if ( angle < ANGLE_MAX_DIFF)
                {
                  sp_tree->updateNode(coord,true);
                } 
            }  
        } 
      else 
        {
          free++;
        }
    }
  ROS_INFO("Extracted map size: %i (%i free, and %i occupied leaf nodes were discarded)", supported, free, occupied - supported);
  return sp_tree;
}

bool map_points_to_keys(soma_pcl_segmentation::GetPCLOctomapMapping::Request  &req,
                        soma_pcl_segmentation::GetPCLOctomapMapping::Response &res){
  
  OcTree* input_tree;
  AbstractOcTree* tree = octomap_msgs::fullMsgToMap(req.octomap);
  if (!tree){
    ROS_ERROR("Failed to recreate octomap");
    return false;
  }
  OcTree* octree = dynamic_cast<OcTree*>(tree);
  
  if (octree){
    ROS_INFO("Map received (%zu nodes, %f m res)", octree->size(), octree->getResolution());
    input_tree = extract_supporting_planes(octree);
  } else{
    ROS_ERROR("No map received!");
    input_tree = NULL;
  }

  for (int i=0; i<req.points.size(); ++i)
    {
      
        point3d coord (req.points[i].x, 
                       req.points[i].y, 
                       req.points[i].z);
        
        const OcTreeKey key = input_tree->coordToKey(coord);
        OcTreeKey::KeyHash computeHash;
        unsigned short int hash = computeHash(key);
        res.keys.push_back(hash);
    }
  ROS_INFO("Send response (%i)", res.keys.size());
  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "pcl_octomap_mapper_service");
  ros::NodeHandle nh("~");
  ros::ServiceServer pcl_octomap_mapper_service = nh.advertiseService("pcl_octomap_mapper", map_points_to_keys);
  ROS_INFO("Semantic segmentation service ready.");
  ros::spin();

  return 0;
}
