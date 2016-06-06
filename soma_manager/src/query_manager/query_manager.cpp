#include <ros/ros.h>
#include <mongodb_store/message_store.h>
#include "querybuilder.h"
#include <soma_manager/SOMA2QueryObjs.h>
#include <soma_map_manager/MapInfo.h>
#include <QCollator>
#include <QDir>
#include <QTextStream>
#include <QDebug>
#include <QString>

std::string objectsdbname="soma2data";
std::string roidbname="soma2data";
std::string objectscollectionname="soma2";
std::string map_name="kthfloor6";

struct SOMA2TimeLimits{

    int maxtimestep;
    int mintimestep;

    long mintimestamp;
    long maxtimestamp;

};

std::vector<soma2_msgs::SOMA2ROIObject> fetchSOMA2ROIs()
{
    std::vector<soma2_msgs::SOMA2ROIObject> res;

    ros::NodeHandle nl;

    mongodb_store::MessageStoreProxy soma2store(nl,"soma2_roi",roidbname);

    mongo::BSONObjBuilder builder;

    builder.append("map_name",map_name);



    // qDebug()<<str;

    // builder.appendElements(mongo::fromjson(str.toStdString()));


    std::vector<boost::shared_ptr<soma2_msgs::SOMA2ROIObject> > rois;


    soma2store.query(rois,builder.obj());

    nl.shutdown();

    if(rois.size() > 0)
    {
        for(auto &roi:rois)
        {
            res.push_back(*roi);
        }
    }


    return res;
}

std::vector<std::vector<std::string> > fetchSOMA2ObjectTypesIDs()
{
    ros::NodeHandle nl;

    std::vector< std::vector<std::string> > result;

    mongodb_store::MessageStoreProxy soma2store(nl,objectscollectionname,objectsdbname);

    std::vector<boost::shared_ptr<soma2_msgs::SOMA2Object> >  soma2objects;

    std::vector<std::string> soma2types;

    std::vector<std::string> soma2ids;


    // Query all objects,
    soma2store.query(soma2objects);

    // List that stores the object types
    QStringList typesls;

    // List that stores the object ids
    QStringList idsls;

    nl.shutdown();

    // If we have any objects
    if(soma2objects.size()>0)
    {


        for(int i = 0; i < soma2objects.size(); i++)
        {
            QString str;

            QString str2;



            //   spr = soma2objects[i];

            str.append(QString::fromStdString(soma2objects[i]->type));

            str2.append(QString::fromStdString(soma2objects[i]->id));


            typesls.append(str);

            idsls.append(str2);



            //std::cout<<soma22objects[i].use_count()<<std::endl;


        }
    }

    //  soma2objects.clear();


    // Remove duplicate names
    typesls.removeDuplicates();

    // Sort the types
    // typesls.sort(Qt::CaseInsensitive);


    QCollator collator;
    collator.setNumericMode(true);

    std::sort(
                typesls.begin(),
                typesls.end(),
                [&collator](const QString &file1, const QString &file2)
    {
        return collator.compare(file1, file2) < 0;
    });




    foreach(QString st, typesls)
    {

        soma2types.push_back(st.toStdString());
    }




    // Remove duplicate names
    idsls.removeDuplicates();



    std::sort(
                idsls.begin(),
                idsls.end(),
                [&collator](const QString &file1, const QString &file2)
    {
        return collator.compare(file1, file2) < 0;
    });


    QString st;


    foreach(st, idsls)
    {
        soma2ids.push_back(st.toStdString());

    }


    result.push_back(soma2types);
    result.push_back(soma2ids);

    return result;

}
SOMA2TimeLimits getSOMA2CollectionMinMaxTimestep()
{


    SOMA2TimeLimits limits;
    limits.mintimestamp = -1;
    limits.mintimestep = -1;
    limits.maxtimestamp = -1;
    limits.maxtimestep = -1;

    ros::NodeHandle nl;

    mongodb_store::MessageStoreProxy soma2store(nl,objectscollectionname,objectsdbname);

    mongo::BSONObjBuilder builder;

    builder.append("$natural",-1);

    std::vector<boost::shared_ptr<soma2_msgs::SOMA2Object> > soma2objects;

    soma2store.query(soma2objects,mongo::BSONObj(),builder.obj(),false,1);


    if(soma2objects.size() > 0){
        limits.maxtimestep = soma2objects[0]->timestep;
        limits.maxtimestamp = soma2objects[0]->logtimestamp;
    }
    //std::cout<<soma2objects[0]->timestep<<std::endl;

    soma2objects.clear();
    soma2store.query(soma2objects,mongo::BSONObj(),mongo::BSONObj(),false,1);

    if(soma2objects.size() > 0)
    {

        limits.mintimestep = soma2objects[0]->timestep;
        limits.mintimestamp = soma2objects[0]->logtimestamp;
    }

    return limits;

}
std::vector<soma2_msgs::SOMA2Object> querySOMA2Objects(const mongo::BSONObj &queryobj)
{

    ros::NodeHandle nl;

    mongodb_store::MessageStoreProxy soma2store(nl,objectscollectionname,objectsdbname);


    std::vector<soma2_msgs::SOMA2Object> res;



    std::vector<boost::shared_ptr<soma2_msgs::SOMA2Object> > soma2objects;



    soma2store.query(soma2objects,queryobj);


    if(soma2objects.size() > 0)
    {
        for(auto &labelled_object:soma2objects)
        {
            res.push_back(*labelled_object);
            //qDebug()<<labelled_object.use_count;
        }

    }


    qDebug()<<"Query returned"<<res.size()<<"objects";

    return res;


}


bool handleQueryRequests(soma_manager::SOMA2QueryObjsRequest & req, soma_manager::SOMA2QueryObjsResponse& resp)
{
    mongo::BSONObjBuilder mainbuilder;

    // We are building a SOMA2Object Query
    if(req.query_type == 0)
    {
        // If dates are used
        if(req.usedates)
        {

            int mode = 0;

            if(req.lowerdate > 0 && req.upperdate > 0)
            {
                mode = 2;

            }
            else if(req.upperdate > 0)
                mode = 1;

            mongo::BSONObj bsonobj = QueryBuilder::buildSOMA2DateQuery(req.lowerdate,req.upperdate,mode);

            mainbuilder.appendElements(bsonobj);

        }
        // If timestep is used instead
        else if(req.usetimestep)
        {
            qDebug()<<"I am here at timestep"<<req.timestep<<req.usetimestep;
            mongo::BSONObj timestepobj = QueryBuilder::buildSOMA2TimestepQuery(req.timestep);

            mainbuilder.appendElements(timestepobj);


        }
        // If any of the time limits are used
        if(req.uselowertime || req.useuppertime)
        {

            int mode = 0;
            if(req.uselowertime && req.useuppertime)
                mode = 2;
            else if(req.useuppertime)
                mode=1;

            mongo::BSONObj bsonobj = QueryBuilder::buildSOMA2TimeQuery(req.lowerhour,req.lowerminutes,req.upperhour,req.upperminutes,mode);

            mainbuilder.appendElements(bsonobj);


        }
        // If weekday limit is used
        if(req.useweekday)
        {
            mongo::BSONObj bsonobj = QueryBuilder::buildSOMA2WeekdayQuery(req.weekday);


            mainbuilder.appendElements(bsonobj);
        }

        // If roi is used
        if(req.useroi)
        {

            ros::NodeHandle nl;

            mongodb_store::MessageStoreProxy soma2store(nl,"soma2_roi",roidbname);

            mongo::BSONObjBuilder builder;

            builder.append("map_name",map_name);


            std::vector<boost::shared_ptr<soma2_msgs::SOMA2ROIObject> > rois;


            soma2store.query(rois,builder.obj());

            nl.shutdown();



            for(int i  = 0;i < rois.size(); i++)
            {
                soma2_msgs::SOMA2ROIObject roi = *rois[i];

                if(roi.id == req.roi_id)
                {
                    mongo::BSONObj bsonobj = QueryBuilder::buildSOMA2ROIWithinQuery(roi);

                    mainbuilder.appendElements(bsonobj);

                    break;

                }

            }


        }
        // If object ids and/or types are used
        if(req.objectids.size()>0 || req.objecttypes.size() > 0)
        {
            if(req.objectids.size() > 0 && req.objecttypes.size() > 0)
            {
                qDebug()<<req.objectids.size()<<req.objecttypes.size();
                if(req.objectids[0] != "" && req.objecttypes[0] != "")
                {

                    std::vector<std::string> list;

                    std::vector<std::string> fieldnames;
                    std::vector<int> objectIndexes;
                    fieldnames.push_back("id");
                    fieldnames.push_back("type");



                    list.insert(list.end(),req.objectids.begin(),req.objectids.end());

                    objectIndexes.push_back(req.objectids.size());

                    list.insert(list.end(),req.objecttypes.begin(),req.objecttypes.end());

                    objectIndexes.push_back(req.objecttypes.size());

                    mongo::BSONObj bsonobj = QueryBuilder::buildSOMA2StringArrayBasedQuery(list,fieldnames,objectIndexes,"$or");

                    mainbuilder.appendElements(bsonobj);
                }
                else if(req.objecttypes[0] == "")
                {



                    std::vector<std::string> fieldnames;
                    fieldnames.push_back("id");

                    std::vector<int> objectIndexes;
                    objectIndexes.push_back(req.objectids.size());

                    std::vector<std::string> list;

                    //   list.insert(list.end(),req.objectids.data()->begin(),req.objectids.data()->end());


                    mongo::BSONObj bsonobj = QueryBuilder::buildSOMA2StringArrayBasedQuery(req.objectids,fieldnames,objectIndexes,"$or");



                    mainbuilder.appendElements(bsonobj);

                }
                else if(req.objectids[0] == "")
                {
                    std::vector<std::string> fieldnames;
                    fieldnames.push_back("type");

                    std::vector<int> objectIndexes;
                    objectIndexes.push_back(req.objecttypes.size());

                    mongo::BSONObj bsonobj = QueryBuilder::buildSOMA2StringArrayBasedQuery(req.objecttypes,fieldnames,objectIndexes,"$or");

                    mainbuilder.appendElements(bsonobj);

                }

            }
            else if(req.objectids.size() > 0)
            {
                std::vector<std::string> fieldnames;
                fieldnames.push_back("id");

                std::vector<int> objectIndexes;
                objectIndexes.push_back(req.objectids.size());

                std::vector<std::string> list;

                //   list.insert(list.end(),req.objectids.data()->begin(),req.objectids.data()->end());


                mongo::BSONObj bsonobj = QueryBuilder::buildSOMA2StringArrayBasedQuery(req.objectids,fieldnames,objectIndexes,"$or");



                mainbuilder.appendElements(bsonobj);

            }
            else if(req.objecttypes.size() > 0)
            {
                std::vector<std::string> fieldnames;
                fieldnames.push_back("type");

                std::vector<int> objectIndexes;
                objectIndexes.push_back(req.objecttypes.size());

                mongo::BSONObj bsonobj = QueryBuilder::buildSOMA2StringArrayBasedQuery(req.objecttypes,fieldnames,objectIndexes,"$or");

                mainbuilder.appendElements(bsonobj);

            }



        }

        if(mainbuilder.len() > 0){

            mongo::BSONObj tempObject = mainbuilder.obj();

           // qDebug()<<QString::fromStdString(tempObject.jsonString());

            std::vector< soma2_msgs::SOMA2Object > soma2objects =  querySOMA2Objects(tempObject);

            resp.objects = soma2objects;
            resp.queryjson = tempObject.jsonString();

        }


    }



    // Handle Query for type and ids
    else if(req.query_type == 1)
    {
        std::vector<std::vector<std::string> > res = fetchSOMA2ObjectTypesIDs();
        if(res.size()>=2)
        {
            resp.types = res[0];
            resp.ids = res[1];
        }

    }
    // Handle Query for rois
    else if(req.query_type == 2)
    {
        std::vector<soma2_msgs::SOMA2ROIObject> res = fetchSOMA2ROIs();

        resp.rois = res;

    }
    // Handle Query for timelimits
    else if(req.query_type == 3)
    {
        SOMA2TimeLimits res = getSOMA2CollectionMinMaxTimestep();

        resp.timedatelimits.push_back(res.mintimestep);
        resp.timedatelimits.push_back(res.maxtimestep);
        resp.timedatelimits.push_back(res.mintimestamp);
        resp.timedatelimits.push_back(res.maxtimestamp);

    }



    return true;
}

int main(int argc, char **argv){


    ros::init(argc, argv, "query_manager_node");

    ros::NodeHandle n;


    //std::string mongodbhost;
    //std::string mongodbport;
    std::string roidb;


    if(argc < 2)
    {

        std::cout<<
                    "Running the query_manager_node with default arguments: ObjectsDBName: soma2data, ObjectsCollectionName:soma2, ROIDBName:soma2data"
                 <<std::endl;
        // std::cout << "Not enough input arguments!! Quitting..."<<std::endl;

        //  return -1;

    }
    else
    {
        if(argc > 1){
            objectsdbname = argv[1];
            //mw.rosthread.setSOMA2ObjectsDBName(objectsdbname);

        }
        if(argc >2){
            objectscollectionname = argv[2];
            // mw.rosthread.setSOMA2ObjectsCollectionName(objectscollectionname);
        }
        if(argc >3){
            roidb = argv[3];
            //mw.rosthread.setSOMA2ROIDBName(roidb);
        }

    }

    ros::ServiceClient client = n.serviceClient<soma_map_manager::MapInfo>("soma2/map_info");

    ROS_INFO("Waiting for SOMA Map Service...");

    while(!client.exists() && ros::ok());

    if(!ros::ok())
        return 0;

    soma_map_manager::MapInfo srv;

    client.call(srv);
    ROS_INFO("Received map info. Map Name: %s, Map Unique ID: %s",srv.response.map_name.data(),srv.response.map_unique_id.data());

    map_name = srv.response.map_name;

    ros::ServiceServer service = n.advertiseService("soma2/query_db", handleQueryRequests);
    ROS_INFO("SOMA2 Query Service Ready.");

    ros::spin();

    /*ros::NodeHandle n;

    mongodb_store::MessageStoreProxy soma2store(n,this->objectscollectionname,this->objectsdbname);*/

    //RosThread thread();



    return 0;


}

