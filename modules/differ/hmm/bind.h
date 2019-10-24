#ifndef ROAD_MATCH_SERVICE_BIND_H
#define ROAD_MATCH_SERVICE_BIND_H

#include "data_manager/data_types.h"

#include <list>

class Bind {
public:
    double distance_;
    int index_;
    int pos_index_;
    int s2e_;
    shared_ptr<KDRoad> match_road_;
    shared_ptr<KDCoord> query_point_;
    shared_ptr<KDCoord> snapped_point_;
    double length_to_start_;
    double length_to_end_;
    long route_id_;
};

class CadidatesStep {
public:
    vector<shared_ptr<Bind>> candidates_;
};

class StepList {
public:
    int route_id_;
    list<shared_ptr<CadidatesStep>> step_list_;
};

#endif //ROAD_MATCH_SERVICE_BIND_H