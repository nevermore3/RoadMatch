#ifndef ROAD_MATCH_SERVICE_BIND_H
#define ROAD_MATCH_SERVICE_BIND_H

#include "data_manager/data_types.h"

#include <list>

// 每个匹配对象
class Bind {
public:

    // route上形点到匹配到road的垂直距离
    double distance_;

    // route形点的index
    int index_;

    //基础路网上垂足点的前一个index
    int pos_index_;

    int s2e_;

    //匹配到的基础路网的road
    shared_ptr<KDRoad> match_road_;

    //route的查询形点
    shared_ptr<KDCoord> query_point_;

    //基础路网上的垂足点
    shared_ptr<KDCoord> snapped_point_;

    //垂足点到起始点的距离
    double length_to_start_;

    //垂足点到终止点的距离
    double length_to_end_;

    long route_id_;
};


class CadidatesStep {
public:
    // 每个形点匹配到的候选集合
    vector<shared_ptr<Bind>> candidates_;
};

class StepList {
public:

    int route_id_;
    // 每条route的 每个形点到候选集合
    list<shared_ptr<CadidatesStep>> step_list_;
};

#endif //ROAD_MATCH_SERVICE_BIND_H