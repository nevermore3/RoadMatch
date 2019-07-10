//
// Created by gaoyanhong on 2018/12/18.
//

#include "geometry_util.h"

#include "geos/geom/Coordinate.h"
#include "geos/geom/GeometryFactory.h"
#include "geos/geom/CoordinateSequence.h"
#include "geos/geom/CoordinateArraySequence.h"
#include "geos/geom/Point.h"


#include <mvg/Coordinates.hpp>

using namespace kd::automap;

shared_ptr<LineString> GeometryUtil::CreateLineString(const vector<shared_ptr<KDCoord>> &coords) {

    const geos::geom::GeometryFactory *gf = geos::geom::GeometryFactory::getDefaultInstance();

    //build geos object
    CoordinateSequence *cl = new CoordinateArraySequence();
    for (int j = 0; j < coords.size(); j++) {

        double utmX, utmY;
        char zone[4] = {0};
        Coordinates::ll2utm(coords[j]->lat_, coords[j]->lng_, utmX, utmY, zone);

        cl->add(Coordinate(utmX, utmY, coords[j]->z_));
    }

    shared_ptr<LineString> line(gf->createLineString(cl));

    return line;
}

shared_ptr<LineString> GeometryUtil::CreateLineString(const shared_ptr<KDCoord> start,
                                                      const shared_ptr<KDCoord> end) {

    const geos::geom::GeometryFactory *gf = geos::geom::GeometryFactory::getDefaultInstance();

    //build geos object
    CoordinateSequence *cl = new CoordinateArraySequence();

    double utmX, utmY;
    char zone[4] = {0};
    Coordinates::ll2utm(start->lat_, start->lng_, utmX, utmY, zone);
    cl->add(Coordinate(utmX, utmY, start->z_));

    Coordinates::ll2utm(end->lat_, end->lng_, utmX, utmY, zone);
    cl->add(Coordinate(utmX, utmY, end->z_));

    shared_ptr<LineString> line(gf->createLineString(cl));

    return line;
}

shared_ptr<Polygon> GeometryUtil::CreatePolygon(const vector<shared_ptr<KDCoord>> &coords) {

    const geos::geom::GeometryFactory *gf = geos::geom::GeometryFactory::getDefaultInstance();

    //build geos object
    CoordinateSequence *cl = new CoordinateArraySequence();
    for (int j = 0; j < coords.size(); j++) {

        double utmX, utmY;
        char zone[4] = {0};
        Coordinates::ll2utm(coords[j]->lat_, coords[j]->lng_, utmX, utmY, zone);

        cl->add(Coordinate(utmX, utmY, coords[j]->z_));
    }

    //补充多边形首点
    {
        double utmX, utmY;
        char zone[4] = {0};
        Coordinates::ll2utm(coords[0]->lat_, coords[0]->lng_, utmX, utmY, zone);

        cl->add(Coordinate(utmX, utmY, coords[0]->z_));
    }

    LinearRing *linearRing = gf->createLinearRing(cl);

    shared_ptr<Polygon> polygon(gf->createPolygon(linearRing, NULL));

    return polygon;
}

shared_ptr<Point> GeometryUtil::CreatePoint(const shared_ptr<KDCoord> coord) {

    return CreatePoint(coord.get());
}

shared_ptr<Point> GeometryUtil::CreatePoint(const KDCoord *coord) {

    const geos::geom::GeometryFactory *gf = geos::geom::GeometryFactory::getDefaultInstance();

    double utmX, utmY;
    char zone[4] = {0};
    Coordinates::ll2utm(coord->lat_, coord->lng_, utmX, utmY, zone);

    shared_ptr<Point> point(gf->createPoint(Coordinate(utmX, utmY, coord->z_)));

    return point;
}