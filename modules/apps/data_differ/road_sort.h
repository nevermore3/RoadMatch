#ifndef HDMAP_DIFFER_FUSION_ROAD_SORT_H
#define HDMAP_DIFFER_FUSION_ROAD_SORT_H

#include "util/distance.h"
#include "geos/indexQuadtree.h"
#include "geos/indexStrtree.h"
#include "geos/geom/LineString.h"
#include "data_manager/route_manager.h"

using namespace geos::index::quadtree;
using namespace geos::index::strtree;

class KDRoad;

class QueryRoad
{
public:
    shared_ptr<KDRoad>road_;
    double distance_;
    double start_distance_;
    double end_distance_;
    explicit QueryRoad(shared_ptr<KDRoad>road) : road_(std::move(road)), distance_(DBL_MAX),
                                                 start_distance_(DBL_MAX), end_distance_(DBL_MAX) {}
};

template <typename  T>
class Compare{
public:
    explicit Compare(int flag) : flag_(flag) {}

    bool operator ()(T link1, T link2){
        if (flag_ == 0) {
            return link1->start_distance_ < link2->start_distance_;
        } else if (flag_ == 1) {
            return link1->end_distance_ < link2->end_distance_;
        } else {
            return link1->distance_ < link2->distance_;
        }
    }
private:
    int flag_;
};

template <typename  T>
void FilterRoad(vector<T> &array, int flag)
{
    Compare<T> compare(flag);
    sort(array.begin(), array.end(), compare);
    //保留前count个
    size_t count = 2;
    size_t i = 1;
    auto iter = array.begin();
    while (iter != array.end()) {
        if (flag == 0) {
            if (i > count || (*iter)->start_distance_ == DBL_MAX) {
                iter = array.erase(iter);
            } else {
                i++;
                iter++;
            }
        } else if (flag == 1) {
            if (i > count || (*iter)->end_distance_ == DBL_MAX) {
                iter = array.erase(iter);
            } else {
                i++;
                iter++;
            }
        } else {
            if (i > count || (*iter)->distance_ == DBL_MAX) {
                iter = array.erase(iter);
            } else {
                i++;
                iter++;
            }
        }
    }
}

class QueryRoute
{
public:
    double distance_;
    double start_distance_;
    double end_distance_;
    shared_ptr<Route>route_;
    explicit QueryRoute(shared_ptr<Route> route) : route_(std::move(route)), distance_(DBL_MAX),
                                                   start_distance_(DBL_MAX), end_distance_(DBL_MAX) {}
};

#endif //HDMAP_DIFFER_FUSION_ROAD_SORT_H
