//
// Created by gaoyanhong on 2018/12/18.
//

#ifndef MATCH_CORE_GEO_UTIL_H
#define MATCH_CORE_GEO_UTIL_H

#include "data_manager/data_types.h"

class MatchGeoUtil {

public:

    static bool CoordDirectionSame(const vector<shared_ptr<KDCoord>> &coords_src,
                                   const vector<shared_ptr<KDCoord>> &coords_dst);

    static bool CoordDirectionSame(const geos::geom::Coordinate * coord1, const geos::geom::Coordinate * coord2,
                                   const geos::geom::Coordinate * coord3, const geos::geom::Coordinate * coord4);



    static double CalculateAngle(shared_ptr<KDRoad> road, bool start);

    /**
     * 获得道路上，距离起点或者终点指定距离处的坐标差值点。
     * @param road
     * @param start
     * @param start_pt
     * @param end_pt
     * @param distance
     */
    static void GetCoord(shared_ptr<KDRoad> road, bool start, KDCoord &start_pt, KDCoord &end_pt, double distance);

    static double GetAngleDiff(double angle1, double angle2);

    static bool RoadIsIntersect(shared_ptr<LineString> lineobj, shared_ptr<geos::geom::Geometry> geom_buffer, shared_ptr<LineString> line_check);


    static bool GetIntersectInfo(const shared_ptr<geos::geom::LineString> line,
                                 const geos::geom::Coordinate *ref_pt, int begin_index,
                                 int &node_index, float &node_dist, double &node_z);

    static bool GetPointIndex(const shared_ptr<geos::geom::LineString> line,
                              const geos::geom::Coordinate *coord, int start_index, int &node_index, float &node_dist);


    static bool ValueBetween(double value1, double value2, double value3);

    /**
     * 根据已知点，获得两点之间的插值点
     * @param coord1 参考点1
     * @param coord2 参考点2
     * @param distance 插值点距离参考点1的距离
     * @param coord 差指点坐标
     */
    static void GetDifferenceCoord(const geos::geom::Coordinate &coord1, const geos::geom::Coordinate &coord2,
                                   double distance, geos::geom::Coordinate &coord);

    static bool GetDifferenceCoord(const shared_ptr<geos::geom::LineString> line,
                                   int node_index, float &node_dist, geos::geom::Coordinate &coord);

    static double getDistance(double dx1, double dy1, double dx2, double dy2);

};

#endif //MATCH_CORE_GEO_UTIL_H
