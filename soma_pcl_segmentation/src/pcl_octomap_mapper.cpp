//Ros includes
#include "ros/ros.h"

//STL includes
#include <vector>

//Service includes
#include "soma_pcl_segmentation/GetPCLOctomapMapping.h"

bool map_cloud_to_octomap(soma_pcl_segmentation::GetPCLOctomapMapping::Request  &req,
                          soma_pcl_segmentation::GetPCLOctomapMapping::Response &res){

  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "pcl_octomap_mapper_service");
  ros::NodeHandle nh("~");
  ros::ServiceServer pcl_octomap_mapper_service = nh.advertiseService("pcl_octomap_mapper", map_cloud_to_octomap);
  ROS_INFO("Semantic segmentation service ready.");
  ros::spin();

  return 0;
}
