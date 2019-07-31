//
// Created by ubuntu on 19-7-31.
//

#ifndef HDMAP_DIFFER_FUSION_ROAD_SORT_H
#define HDMAP_DIFFER_FUSION_ROAD_SORT_H

#include "util/distance.h"
#include "geos/indexQuadtree.h"
#include "geos/indexStrtree.h"
#include "geos/geom/LineString.h"

using namespace geos::index::quadtree;
using namespace geos::index::strtree;

class KDRoad;

class QueryRoad
{
public:
    shared_ptr<KDRoad>road_;
    double distances_;
    double start_distance_;
    double end_distance_;
    explicit QueryRoad(shared_ptr<KDRoad>road) : road_(std::move(road)), distances_(DBL_MAX),
                                                 start_distance_(DBL_MAX), end_distance_(DBL_MAX) {}
};


class Compare{
public:
    Compare(int flag) : flag_(flag) {}

    bool operator ()(shared_ptr<QueryRoad> link1, shared_ptr<QueryRoad>link2){
        if (flag_ == 0) {
            return link1->start_distance_ < link2->start_distance_;
        } else if (flag_ == 1) {
            return link1->end_distance_ < link2->end_distance_;
        } else {
            return link1->distances_ < link2->distances_;
        }

    }
private:
    int flag_;
};


#endif //HDMAP_DIFFER_FUSION_ROAD_SORT_H
