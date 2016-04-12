#ifndef QUERYBUILDER_H
#define QUERYBUILDER_H
#include <mongo/bson/bson.h>
#include <mongo/bson/bsonelement.h>
#include <soma2_msgs/SOMA2ROIObject.h>
#include <mongo/util/time_support.h>
#include <mongo/bson/bsontypes.h>
class QueryBuilder
{
public:
    QueryBuilder();

    static mongo::BSONObj buildSOMA2ROIWithinQuery(const soma2_msgs::SOMA2ROIObject& roiobj);

    static mongo::BSONObj buildSOMA2TypeEqualsQuery(const std::vector<std::string>& typelist);

    static mongo::BSONObj buildSOMA2LabelContainsQuery(const std::string& text);

    static mongo::BSONObj buildSOMA2WeekdayQuery(int index);

    static mongo::BSONObj buildSOMA2DateQuery(ulong lowerdate, ulong upperdate, int mode);

    static mongo::BSONObj buildSOMA2TimeQuery(int lowerhour,int lowerminute, int upperhour, int upperminute,  int mode);

    static mongo::BSONObj buildSOMA2TimestepQuery(int timestep);

    static mongo::BSONObj buildSOMA2StringArrayBasedQuery(const std::vector<std::string>& list, std::vector<std::string> fieldnames, std::vector<int> objectIndexes, std::string arrayOperator);

private:
};

#endif // QUERYBUILDER_H
