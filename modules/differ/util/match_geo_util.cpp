//
// Created by gaoyanhong on 2018/12/18.
//

#include "match_geo_util.h"

#include "geom/geo_util.h"

#include "mvg/Coordinates.hpp"


bool MatchGeoUtil::CoordDirectionSame(const vector<shared_ptr<KDCoord>> &coords_src,
                                      const vector<shared_ptr<KDCoord>> &coords_dst) {

    shared_ptr<KDCoord> src_first = coords_src.front();
    shared_ptr<KDCoord> src_last = coords_src.back();
    double angle1 = geo::geo_util::calcAngle(src_first->lng_, src_first->lat_, src_last->lng_, src_last->lat_);

    shared_ptr<KDCoord> dst_first = coords_dst.front();
    shared_ptr<KDCoord> dst_last = coords_dst.back();
    double angle2 = geo::geo_util::calcAngle(dst_first->lng_, dst_first->lat_, dst_last->lng_, dst_last->lat_);

    double angle_diff = angle1 - angle2;
    if (angle_diff < 0) {
        angle_diff = 2 * M_PI + angle_diff;
    }

    if ((angle_diff > M_PI / 2.0) && (angle_diff < M_PI * 3 / 2.0)) {
        return false;
    } else {
        return true;
    }
}

bool MatchGeoUtil::CoordDirectionSame(const geos::geom::Coordinate *coord1, const geos::geom::Coordinate *coord2,
                                      const geos::geom::Coordinate *coord3, const geos::geom::Coordinate *coord4) {

//    double angle1 = geo::geo_util::calcAngle(coord1->x, coord1->y, src_last->lng_, src_last->lat_);
//
//    double angle2 = geo::geo_util::calcAngle(dst_first->lng_, dst_first->lat_, dst_last->lng_, dst_last->lat_);
//
//    double angle_diff = angle1 - angle2;
//    if (angle_diff < 0) {
//        angle_diff = 2 * M_PI + angle_diff;
//    }
//
//    if ((angle_diff > M_PI / 2.0) && (angle_diff < M_PI * 3 / 2.0)) {
//        return false;
//    } else {
//        return true;
//    }

    return true;
}

double MatchGeoUtil::CalculateAngle(shared_ptr<KDRoad> road, bool start) {

    //道路截取20米，不足20米取实际长度
    KDCoord start_pt, end_pt;
    GetCoord(road, start, start_pt, end_pt, 20.0);

    return geo::geo_util::calcAngle(start_pt.lng_, start_pt.lat_, end_pt.lng_, end_pt.lat_);
}

void MatchGeoUtil::GetCoord(shared_ptr<KDRoad> road, bool start, KDCoord &start_pt, KDCoord &end_pt, double distance) {

    const geos::geom::CoordinateSequence *cs = road->line_->getCoordinatesRO();

    double dist_temp = 0.0;

    if (start) {
        start_pt.lng_ = road->points_.front()->lng_;
        start_pt.lat_ = road->points_.front()->lat_;

        for (int i = 0; i < cs->size() - 1; i++) {
            const geos::geom::Coordinate &start = cs->getAt(i);
            const geos::geom::Coordinate &end = cs->getAt(i + 1);

            double dx = start.x - end.x;
            double dy = start.y - end.y;

            double seg_dist = sqrt(dx * dx + dy * dy);
            if (dist_temp + seg_dist < distance) {
                dist_temp += seg_dist;
            } else {
                double len = distance - dist_temp;

                shared_ptr<KDCoord> s_c = road->points_[i];
                shared_ptr<KDCoord> e_c = road->points_[i + 1];

                end_pt.lng_ = len * (e_c->lng_ - s_c->lng_) / seg_dist + s_c->lng_;
                end_pt.lat_ = len * (e_c->lat_ - s_c->lat_) / seg_dist + s_c->lat_;
                return;
            }
        }

        if (dist_temp <= distance) {
            end_pt.lng_ = road->points_.back()->lng_;
            end_pt.lat_ = road->points_.back()->lat_;
        }
    } else {
        start_pt.lng_ = road->points_.back()->lng_;
        start_pt.lat_ = road->points_.back()->lat_;

        for (int i = cs->size() - 1; i > 0; i--) {
            const geos::geom::Coordinate &start = cs->getAt(i);
            const geos::geom::Coordinate &end = cs->getAt(i - 1);

            double dx = start.x - end.x;
            double dy = start.y - end.y;

            double seg_dist = sqrt(dx * dx + dy * dy);
            if (dist_temp + seg_dist < distance) {
                dist_temp += seg_dist;
            } else {
                double len = distance - dist_temp;

                shared_ptr<KDCoord> s_c = road->points_[i];
                shared_ptr<KDCoord> e_c = road->points_[i - 1];

                end_pt.lng_ = len * (e_c->lng_ - s_c->lng_) / seg_dist + s_c->lng_;
                end_pt.lat_ = len * (e_c->lat_ - s_c->lat_) / seg_dist + s_c->lat_;
                return;
            }
        }

        if (dist_temp <= distance) {
            end_pt.lng_ = road->points_.front()->lng_;
            end_pt.lat_ = road->points_.front()->lat_;
        }
    }
}

double MatchGeoUtil::GetAngleDiff(double angle1, double angle2) {
    double angle_diff = abs(angle1 - angle2);
    if (angle_diff > M_PI) {
        angle_diff = 2 * M_PI - angle_diff;
    }

    return angle_diff;
}

bool MatchGeoUtil::RoadIsIntersect(shared_ptr<LineString> lineobj,
                                   shared_ptr<geos::geom::Geometry> geom_buffer,
                                   shared_ptr<LineString> line_check) {

    if(geom_buffer->intersects(line_check.get())){
        return true;
    }
    return false;
}

bool MatchGeoUtil::GetIntersectInfo(const shared_ptr<geos::geom::LineString> line,
                                           const geos::geom::Coordinate *ref_pt, int begin_index,
                                           int &node_index, float &node_dist, double &node_z) {
    if (GetPointIndex(line, ref_pt, begin_index, node_index, node_dist)) {

        geos::geom::Coordinate diff_coord;
        if (GetDifferenceCoord(line, node_index, node_dist, diff_coord)) {

            node_z = diff_coord.z;
            return true;
        }
    }

    return false;
}

bool MatchGeoUtil::
GetPointIndex(const shared_ptr<geos::geom::LineString> line,
              const geos::geom::Coordinate *coord, int start_index, int &node_index, float &node_dist) {
    if (line == nullptr || coord == nullptr) {
        return false;
    }

    const geos::geom::CoordinateSequence *cs = line->getCoordinatesRO();
    int coord_num = cs->size();
    for (int i = start_index; i < coord_num - 1; i++) {
        const geos::geom::Coordinate &coord1 = cs->getAt(i);
        const geos::geom::Coordinate &coord2 = cs->getAt(i + 1);

        if (ValueBetween(coord1.x, coord2.x, coord->x) &&
            ValueBetween(coord1.y, coord2.y, coord->y)) {

            node_index = i;
            node_dist = getDistance(coord1.x, coord1.y, coord->x, coord->y);

            return true;
        }
    }
    return false;
}

bool MatchGeoUtil::GetDifferenceCoord(const shared_ptr<geos::geom::LineString> line,
                                             int node_index, float &node_dist, geos::geom::Coordinate &coord) {

    if (line == nullptr || node_index < 0 || node_index >= line->getNumPoints() - 1) {
        return false;
    }

    const geos::geom::CoordinateSequence *cs = line->getCoordinatesRO();

    const geos::geom::Coordinate &coord1 = cs->getAt(node_index);
    const geos::geom::Coordinate &coord2 = cs->getAt(node_index + 1);

    GetDifferenceCoord(coord1, coord2, node_dist, coord);

    return true;
}

void MatchGeoUtil::GetDifferenceCoord(const geos::geom::Coordinate &coord1,
                                             const geos::geom::Coordinate &coord2,
                                             double distance, geos::geom::Coordinate &coord) {
    double total_dist = getDistance(coord1.x, coord1.y, coord2.x, coord2.y);

    double ratio = distance / total_dist;
    coord.x = ratio * (coord2.x - coord1.x) + coord1.x;
    coord.y = ratio * (coord2.y - coord1.y) + coord1.y;
    coord.z = ratio * (coord2.z - coord1.z) + coord1.z;
}


bool MatchGeoUtil::ValueBetween(double value1, double value2, double value3) {

    if ((value1 <= value3 && value3 <= value2) ||
        (value1 >= value3 && value3 >= value2)) {
        return true;
    }
    return false;
}


double MatchGeoUtil::getDistance(double dx1, double dy1, double dx2, double dy2) {
    double dx = dx2 - dx1;
    double dy = dy2 - dy1;
    double dis = sqrt(dx * dx  + dy * dy);
    return dis;
}



