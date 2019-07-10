//
// Created by liujian on 19-7-9.
//

#ifndef TD_DATA_DIFFER_DATA_TYPES_H
#define TD_DATA_DIFFER_DATA_TYPES_H

#include "geos/geom/LineString.h"
#include "geos/geom/Polygon.h"

#include <string>
#include <memory>
#include <vector>
#include <map>
#include <unordered_map>
#include <float.h>

using namespace std;
using namespace geos::geom;

class KDExtent {
public:
    KDExtent(): xmin_(DBL_MAX),
                ymin_(DBL_MAX),
                xmax_(DBL_MIN),
                ymax_(DBL_MIN){}


    KDExtent(double xmin, double xmax, double ymin, double ymax) {
        xmin_ = xmin;
        xmax_ = xmax;
        ymin_ = ymin;
        ymax_ = ymax;
    }

    double Width() { return xmax_ - xmin_; }

    double Height() { return ymax_ - ymin_; }

    double CenterX() const { return (xmin_ + xmax_) / 2.0; }

    double CenterY() const { return (ymin_ + ymax_) / 2.0; }

    void ExpandToInclude(double x, double y) {
        if(xmin_ > x) xmin_ = x;
        if(ymin_ > y) ymin_ = y;
        if(xmax_ < x) xmax_ = x;
        if(ymax_ < y) ymax_ = y;
    }
public:
    double xmin_;
    double xmax_;
    double ymin_;
    double ymax_;
};

class KDCoord {
public:
    double lng_;
    double lat_;
    double z_;
};

class KDRoad;

class KDRoadNode {
public:
    KDRoadNode() = default;

    int GetConnNum() { return from_roads_.size() + to_roads_.size(); }

public:
    int64_t id_;

    string mesh_id_;

    KDCoord coord_;

    int32_t boundary_;

    vector<shared_ptr<KDRoad>> from_roads_;

    vector<shared_ptr<KDRoad>> to_roads_;

    //跨mesh拓扑关系
    string adj_mesh_id_;
    int32_t adj_id_;
};


class KDRoad {
public:
    int64_t id_;

    int64_t f_node_id_;

    int64_t t_node_id_;

    int32_t direction_;

    int32_t road_type_;

    int32_t road_class_;

    int8_t form_way_;

    vector<shared_ptr<KDCoord>> points_;
    shared_ptr<LineString> line_;

    string mesh_id_;
    string road_name_;

    double length_;
    ~KDRoad() {
        vector<shared_ptr<KDCoord>>().swap(points_);
    }
};

#endif //TD_DATA_DIFFER_DATA_TYPE_H
