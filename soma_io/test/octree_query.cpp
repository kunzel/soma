#include <ros/ros.h>
#include <mongodb_store/message_store.h>
#include <boost/foreach.hpp>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <soma_msgs/PickledOctomap.h>

using namespace std;
int main(int argc, char** argv)
{
    //ros node init
    ros::init(argc, argv, "octree_query_test");
    ros::NodeHandle nh;

    cout<<"connecting to DB..."<<endl;
    //setup connection to mongodb
    mongodb_store::MessageStoreProxy messageStore(nh, "ws_observations","message_store");

    cout<<"looking for 'octomap_msgs::Octomap' type data..."<<endl;
    //query specify type and return all data that meet the requirement to vector
    //vector<boost::shared_ptr<octomap_msgs::Octomap> > results;
    vector<boost::shared_ptr<soma_msgs::PickledOctomap> > results;
    //messageStore.query<octomap_msgs::Octomap>(results);
    messageStore.query<soma_msgs::PickledOctomap>(results);
    cout<<"found "<<results.size()<<" results"<<endl;

    if (results.size()<=0)
    {
        cout<<"no result found, exit program..."<<endl;
        exit(-1);
    }
    cout<<"loading octree data..."<<endl;
    //load data form msg in mongodb
    //octomap_msgs::Octomap msg;
    soma_msgs::PickledOctomap msg;
    msg = *results[0];
    octomap::OcTree* octree = new octomap::OcTree(msg.resolution);
    std::stringstream datastream;
    assert(msg.data.size() > 0);
    datastream.write((const char*) &msg.data[0], msg.data.size());
    octree->readBinaryData(datastream);

    //test bbx
    cout<<"testing binding box..."<<endl;
    double max_x,max_y,max_z;
    double min_x,min_y,min_z;
    octree->getMetricMax(max_x,max_y,max_z);
    cout<<max_x<<" "<<max_y<<" "<<max_z<<endl;
    octree->getMetricMin(min_x,min_y,min_z);
    cout<<min_x<<" "<<min_y<<" "<<min_z<<endl;
    octomap::point3d max_p(max_x, max_y, max_z);
    octomap::point3d min_p(min_x, min_y, min_z);
    octree->setBBXMax(max_p);
    octree->setBBXMin(min_p);
    octree->writeBinary("output.bt");

    cout<<octree->getBBXBounds()<<endl;
    octomap::point3d test(1,0.94,3);
    cout<<octree->inBBX(test)<<endl;

    cout<<"boost usage example ..."<<endl;
    //check for if point is convered by observation
    octomap::point3d query_point(1,0.84,3);
    //BOOST_FOREACH(boost::shared_ptr<octomap_msgs::Octomap> oct, results)
    BOOST_FOREACH(boost::shared_ptr<soma_msgs::PickledOctomap> oct, results)
    {
        octomap::OcTree* checker = new octomap::OcTree(oct->resolution);

        std::stringstream datastream;
        //assert(oct->data.size()>0);
        assert(oct->pickled_data.size()>0);
        //datastream.write((const char*) &oct->data[0], oct->data.size());
        datastream.write((const char*) &oct->data[0], oct->data.size());
        checker->readBinaryData(datastream);
        checker->getMetricMax(max_x, max_y, max_z);
        checker->getMetricMin(min_x, min_y, min_z);
        octomap::point3d max_p(max_x, max_y, max_z);
        octomap::point3d min_p(min_x, min_y, min_z);
        checker->setBBXMax(max_p);
        checker->setBBXMin(min_p);

        if(checker->inBBX(query_point))
            ROS_INFO_STREAM("query point: "<<query_point<<" in observation range: "<<max_p<<min_p);
        else
            ROS_INFO_STREAM("query point: "<<query_point<<" NOT in observation range: " <<max_p<<min_p);

    }

    return 0;
}
