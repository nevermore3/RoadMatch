//
// Created by ubuntu on 19-7-30.
//

#include "road_match.h"
#include "data_manager/mesh_manager.h"
#include "data_manager/diff_data_manager.h"
#include "util/geometry_util.h"
#include "util/distance.h"

#include "glog/logging.h"
#include "glog/log_severity.h"
#include "geos/geom/Coordinate.h"
#include "geos/geom/GeometryFactory.h"
#include "geos/geom/CoordinateSequence.h"
#include "geos/geom/CoordinateArraySequence.h"
#include "geos/geom/Point.h"
#include "global_cache.h"
#include "data_manager/route_manager.h"
#include "pathengine/path_engine.h"
#include "output/shp_output.h"
#include "hmm/bind.h"
#include "hmm/viterbi.h"

#include <shp/ShpData.hpp>
#include <geom/geo_util.h>

#include <iostream>
#include <string>
#include <algorithm>
#include <sys/types.h>
#include <sys/stat.h>
using namespace std;

bool RoadMatch::MatchProcess()
{
    cout<<"Start Match Process"<<endl;
    MeshManager *src = MeshManager::GetInstance();
    DiffDataManager *dest = DiffDataManager::GetInstance();

    RouteManager *routeManager = RouteManager::GetInstance();

    // 查找新增的road
    for (auto route : routeManager->routes_) {
        shared_ptr<Route>routeObj = route.second;
        double bufferSize = 20;
        vector<void *>temp;
        shared_ptr<LineString>searchLine = GeometryUtil::CreateLineString(routeObj->points_);
        shared_ptr<geos::geom::Geometry> geom_buffer(searchLine->buffer(bufferSize));
        src->strtree_->query(geom_buffer->getEnvelopeInternal(), temp);

        vector<shared_ptr<KDRoad>>queryObjs;

        for (auto i : temp) {
            auto *road = static_cast<KDRoad *>(i);
            queryObjs.push_back(src->meshs_[road->mesh_id_]->roads_[road->id_]);
        }

        if (queryObjs.empty()) {
            //TODO 新增

        } else {
            if (routeObj->id_ == 371) {
                MatchRoute(routeObj);
            }

        }

    }

}


void RoadMatch::MatchRoute(shared_ptr<Route> route)
{
    PathEngine engine;
    engine.SetSearchCount(50);
    vector<shared_ptr<KDCoord>> dense_coord_list;
//    double angle = geo::geo_util::calcAngle(136.232323, 39.0,
//                                            136.232323, 39.1);

    for(size_t i = 0; i < route->points_.size() - 1; ++i) {
        auto coord1 = route->points_[i];
        auto coord2 = route->points_[i+1];
        if(i == 0)
            dense_coord_list.emplace_back(coord1);
        if(Distance::distance(coord1, coord2) < 1000) {
            dense_coord_list.emplace_back(coord2);
            continue;
        }
        double angle = geo::geo_util::calcAngle(coord1->lng_, coord1->lat_,
                                                coord2->lng_, coord2->lat_);
        bool bBreak = false;
        int cnt = 0;
        while(!bBreak) {
            cnt++;
            shared_ptr<KDCoord> coord = make_shared<KDCoord>();
            coord->lng_ = coord1->lng_ + cnt * 10.0 *cos(angle)/110000;
            coord->lat_ = coord1->lat_ + cnt * 10.0 *sin(angle)/110000;
            dense_coord_list.emplace_back(coord);

            if(Distance::distance(coord, coord2) < 1000)
                bBreak = true;
        }
        dense_coord_list.emplace_back(coord2);

    }

    //IManager* mesh_manage_ = data_manager.base_data_manager_;
    IManager* mesh_manage_ = MeshManager::GetInstance();

    list<StepList> all_steps;
    StepList stepList;
    for (size_t index = 0; index < dense_coord_list.size(); index++) {
        shared_ptr<KDCoord> coord = dense_coord_list[index];

        shared_ptr<Point> point(GeometryUtil::CreatePoint(coord));
        double query_buffer = Distance::GetDegreeDistance(30.0, coord->lng_, coord->lat_);
        shared_ptr<geos::geom::Geometry> geom_buffer(point->buffer(30.0));
        vector<void *> queryObjs;
        mesh_manage_->strtree_->query(geom_buffer->getEnvelopeInternal(), queryObjs);

        if (queryObjs.size() == 0) {
            continue;
        }
        //构建可能的候选项
        shared_ptr<CadidatesStep> step = make_shared<CadidatesStep>();
        bool valid = false;
        for (auto &record : queryObjs) {
            KDRoad *road = (KDRoad *) record;
            int pos_index;
            shared_ptr<KDCoord> foot = make_shared<KDCoord>();
            double distance = Distance::distance(coord, road->points_, foot, &pos_index);
            if (distance > 30.0 * 100)
                continue;

            shared_ptr<Bind> bind = make_shared<Bind>();
            bind->query_point_ = dense_coord_list[index];
            bind->snapped_point_ = foot;
            bind->distance_ = distance;
            bind->index_ = (int) index;
            bind->pos_index_ = pos_index;
            bind->length_to_start_ = geo::geo_util::getDistance(foot->lng_, foot->lat_,
                                                                road->points_[0]->lng_,
                                                                road->points_[0]->lat_);
            bind->length_to_end_ = geo::geo_util::getDistance(foot->lng_, foot->lat_,
                                                              road->points_[road->points_.size() - 1]->lng_,
                                                              road->points_[road->points_.size() - 1]->lat_);

            bind->match_road_ = mesh_manage_->GetRoad(road->mesh_id_, road->id_);
            bind->s2e_ = -1;
            valid = true;
            step->candidates_.emplace_back(bind);
        }

        if (valid) {
            sort(step->candidates_.begin(), step->candidates_.end(),
                 [](const shared_ptr<Bind> bind1, const shared_ptr<Bind> &bind2) {
                     if (bind1->distance_ < bind2->distance_) {
                         return true;
                     } else {
                         return false;
                     }
                 });

//            while (step->candidates_.size() > 5) {
//                shared_ptr<Bind> bind = step->candidates_.back();
//                if (bind->distance_ > 2000)
//                    step->candidates_.pop_back();
//                else
//                    break;
//            }

            stepList.step_list_.emplace_back(step);
        } else {
            LOG(WARNING) << "Can not find the close roads: " << index << endl;
        }
    }

    all_steps.emplace_back(stepList);
    cout<<"result: "<<route->roads_.size()<<endl;
    list<shared_ptr<KDRoad>> res_list;
    Viterbi viterbi;
    viterbi.Compute(stepList.step_list_, false, res_list, mesh_manage_);

    string output_path = GlobalCache::GetInstance()->out_path();
    string outputPath = output_path + "/test";

    if (access(outputPath.c_str(), F_OK) == -1) {
        if (mkdir(outputPath.c_str(), 0755) == -1) {
            LOG(ERROR)<<"mkdir output error !!!";
            return;
        }
    }
    //ShpOutPut::OutPutLinkList(outputPath + "/path1", result);
    ShpOutPut::OutPutLinkList(outputPath + "/path2", res_list);
}