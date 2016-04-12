#include "querybuilder.h"

QueryBuilder::QueryBuilder()
{


}
mongo::BSONObj QueryBuilder::buildSOMA2LabelContainsQuery(const std::string &text)
{

    mongo::BSONObjBuilder builder;

    mongo::BSONObjBuilder regexbuilder;

    regexbuilder.append("$regex",text);


    builder.append("type",regexbuilder.obj());

    return builder.obj();



}
mongo::BSONObj QueryBuilder::buildSOMA2DateQuery(ulong lowerdate, ulong upperdate, int mode)
{
    mongo::BSONObjBuilder builder;

    mongo::BSONObjBuilder builder2;

    // std::cout<<mode<<std::endl;

    if(mode == 0)
    {

        mongo::Date_t ld = lowerdate;

        builder.appendDate("$gte",ld);



    }
    else if(mode == 1)
    {

        mongo::Date_t ud = upperdate;

        builder.appendDate("$lte",ud);



    }
    else if(mode == 2)
    {
        mongo::Date_t ld = lowerdate;

        builder.appendDate("$gte",ld);

        mongo::Date_t ud = upperdate;

        builder.appendDate("$lte",ud);

    }

    //  qDebug()<<QString::fromStdString(builder.obj().jsonString());

    builder2.append("timestamp",builder.obj());

    // obj<<"timestamp"<<builder.ob();


    return builder2.obj();

}

mongo::BSONObj QueryBuilder::buildSOMA2TimestepQuery(int timestep)
{
    mongo::BSONObjBuilder builder;

    builder.append("timestep",timestep);

    return builder.obj();

}

/*mongo::BSONObj QueryBuilder::buildSOMA2TypeEqualsQuery(const std::vector<std::string>& typelist)
{


    mongo::BSONArrayBuilder arrbuilder;

    for(int i = 0; i < typelist.size(); i++)
    {
        mongo::BSONObjBuilder builder;

        builder.append("type",typelist[i].data());

        arrbuilder.append(builder.obj());

    }

    mongo::BSONObjBuilder builder;

    builder.append("$or",arrbuilder.arr());


    return builder.obj();

}*/
mongo::BSONObj QueryBuilder::buildSOMA2StringArrayBasedQuery(const std::vector<std::string> &list, std::vector<std::string> fieldnames, std::vector<int> objectIndexes, std::string arrayOperator)
{

    std::vector<int> realIndexes;
    realIndexes.push_back(0);

    realIndexes.insert(realIndexes.end(),objectIndexes.begin(),objectIndexes.end());

    mongo::BSONArrayBuilder arrbuilder;

    for(int j = 0 ;j < fieldnames.size(); j++){

        for(int i = realIndexes[j]; i < realIndexes[j]+objectIndexes[j]; i++)
        {
            mongo::BSONObjBuilder builder;

            builder.append(fieldnames[j],list[i].data());

            arrbuilder.append(builder.obj());

        }
    }
    mongo::BSONObjBuilder builder;

    builder.append(arrayOperator,arrbuilder.arr());


    return builder.obj();


}
mongo::BSONObj QueryBuilder::buildSOMA2TimeQuery(int lowerhour,int lowerminute, int upperhour, int upperminute,  int mode)
{
    mongo::BSONObjBuilder builder;

    mongo::BSONObjBuilder hourbuilder;
    mongo::BSONObjBuilder lowerhourbuilder;
    mongo::BSONObjBuilder upperhourbuilder;

    mongo::BSONObjBuilder minutebuilder;
    mongo::BSONObjBuilder lowerminutebuilder;


    // When only lower time is selected
    if(mode == 0)
    {
        int totalminutes = lowerhour*60 + lowerminute;


        lowerhourbuilder.append("$gte",totalminutes);
        hourbuilder.append("logtimeminutes",lowerhourbuilder.obj());

        /*   lowerminutebuilder.append("$gte",lowerminute);

        minutebuilder.append("logminute",lowerminutebuilder.obj());*/

        builder.appendElements(hourbuilder.obj());
        /*  builder.appendElements(minutebuilder.obj());*/


    }

    // When only upper time is selected
    else if(mode == 1)
    {
        int totalminutes = upperhour*60 + upperminute;


        upperhourbuilder.append("$lte",totalminutes);
        hourbuilder.append("logtimeminutes",upperhourbuilder.obj());


        builder.appendElements(hourbuilder.obj());


    }

    // When both time is selected
    else if(mode == 2)
    {
        int lowertotalminutes = lowerhour*60 + lowerminute;


        int uppertotalminutes = upperhour*60 + upperminute;

        lowerhourbuilder.append("$gte",lowertotalminutes);
        lowerhourbuilder.append("$lte",uppertotalminutes);

        hourbuilder.append("logtimeminutes",lowerhourbuilder.obj());

        builder.appendElements(hourbuilder.obj());



    }

    return builder.obj();

}
mongo::BSONObj QueryBuilder::buildSOMA2WeekdayQuery(int index)
{
    // std::stringstream ss;
    //ss<<index;

    mongo::BSONObjBuilder builder;

    builder.append("logday",index);

    return builder.obj();


}

mongo::BSONObj QueryBuilder::buildSOMA2ROIWithinQuery(const soma2_msgs::SOMA2ROIObject &roiobj)
{

    mongo::BSONArrayBuilder b;

    //BSON_ARRAY("\"coordinates\""<<arr);

    for(int i = 0; i < roiobj.geoposearray.poses.size(); i++){

        mongo::BSONArrayBuilder b2;

        geometry_msgs::Pose apose = roiobj.geoposearray.poses[i];


        b2.append(apose.position.x);
        b2.append(apose.position.y);

        b.append(b2.arr());
    }


    mongo::BSONArrayBuilder b3;


    b3.append(b.arr());

    // qDebug()<<QString::fromStdString(b3.obj().jsonString());

    mongo::BSONObjBuilder builder;

    builder.appendArray("coordinates",b3.arr());
    builder.append("type","Polygon");

    mongo::BSONObjBuilder builder2;
    builder2.appendElements(builder.obj());

    mongo::BSONObjBuilder builder3;

    builder3.append("$geometry",builder2.obj());

    //  qDebug()<<QString::fromStdString(builder3.obj().jsonString());

    mongo::BSONObjBuilder builder4;

    builder4.append("$geoWithin",builder3.obj());

    // qDebug()<<QString::fromStdString(builder4.obj().jsonString());



    mongo::BSONObjBuilder builder5;

    builder5.append("geoloc",builder4.obj());


    return builder5.obj();

}
