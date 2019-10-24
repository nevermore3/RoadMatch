#ifndef MATCH_CORE_GEOMETRY_UTIL_H
#define MATCH_CORE_GEOMETRY_UTIL_H

#include "geos/geom/LineString.h"
#include "geos/geom/Polygon.h"
#include "geos/geom/Point.h"

using namespace geos::geom;

#include "data_manager/data_types.h"

class GeometryUtil {

public:
    static shared_ptr<LineString> CreateLineString(const vector<shared_ptr<KDCoord>> &coords);

    static shared_ptr<LineString> CreateLineString(const shared_ptr<KDCoord> start, const shared_ptr<KDCoord> end);

    static shared_ptr<Polygon> CreatePolygon(const vector<shared_ptr<KDCoord>> &coords);

    static shared_ptr<Point> CreatePoint(const shared_ptr<KDCoord> coord);

    static shared_ptr<Point> CreatePoint(const KDCoord * coord);

};

#endif //MATCH_CORE_GEOMETRY_UTIL_H
