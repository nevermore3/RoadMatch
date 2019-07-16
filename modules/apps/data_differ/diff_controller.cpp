//
// Created by liujian on 19-7-10.
//

#include <geom/geo_util.h>
#include <output/shp_output.h>
#include "diff_controller.h"

#include "data_manager/data_types.h"
#include "data_manager/data_manager.h"
#include "data_manager/mesh_manager.h"
#include "data_manager/diff_data_manager.h"
#include "pathengine/path_engine.h"
#include "util/distance.h"
#include "util/geometry_util.h"
#include "hmm/bind.h"
#include "hmm/viterbi.h"
#include "global_cache.h"

#include "glog/logging.h"

bool DiffController::Differing() {
    DataManager data_manager;
    data_manager.extent_ =  KDExtent(115.384, 117.5, 39.322, 41.2);
    data_manager.base_data_manager_ = MeshManager::GetInstance();

    data_manager.diff_data_manager_ = DiffDataManager::GetInstance();

    if(!data_manager.LoadData()) {
        return false;
    }

    shared_ptr<KDRoad> road_src = data_manager.diff_data_manager_->GetRoad("478_451", 56100);
    shared_ptr<KDCoord> src_coord = make_shared<KDCoord>();
    // 116.48254668, 39.8655617774
    src_coord->lng_ =  116.482175;
    src_coord->lat_ =  39.86417;

    shared_ptr<KDRoad> road_dst = data_manager.diff_data_manager_->GetRoad("482_451", 4004);
    shared_ptr<KDCoord> des_coord = make_shared<KDCoord>();
    des_coord->lng_ =  116.468003601;
    des_coord->lat_ =  40.238352272;

//    shared_ptr<KDRoad> road_src = data_manager.base_data_manager_->GetRoad("J50F005020", 1887);
//    shared_ptr<KDCoord> src_coord = make_shared<KDCoord>();
//    src_coord->lng_ =  116.40222222222;
//    src_coord->lat_ =  39.640555555556;
//
//    shared_ptr<KDRoad> road_dst = data_manager.base_data_manager_->GetRoad("J50F004019", 12888);
//    shared_ptr<KDCoord> des_coord = make_shared<KDCoord>();
//    des_coord->lng_ =  116.36333333333;
//    des_coord->lat_ =  39.721944444444;

    PathEngine engine;
    engine.SetSearchCount(50000);
    std::list<shared_ptr<KDRoad>> result;
    engine.FindPath(data_manager.diff_data_manager_, -1, road_src, src_coord, road_dst, des_coord, result);

    vector<shared_ptr<KDCoord>> coord_list;

    auto road_it = result.begin();
    auto next_it = road_it;
    while(road_it != result.end()) {
        ++next_it;
        if(next_it == result.end()) {
            break;
        }
        PathEngine::ConnType conn_type =
                PathEngine::GetConnectType(data_manager.diff_data_manager_, (*road_it), (*next_it));

        if(conn_type == PathEngine::UN_CONN)
            break;

        if(conn_type == PathEngine::TAIL_HEAD) {
            if (road_it == result.begin()) {
                coord_list.insert(coord_list.end(), (*road_it)->points_.begin(), (*road_it)->points_.end());
            }
            coord_list.insert(coord_list.end(), ++((*next_it)->points_.begin()), (*next_it)->points_.end());
        } else if (conn_type == PathEngine::TAIL_TAIL) {
            if (road_it == result.begin()) {
                coord_list.insert(coord_list.end(), (*road_it)->points_.begin(), (*road_it)->points_.end());
            }
            coord_list.insert(coord_list.end(), ++((*next_it)->points_.rbegin()), (*next_it)->points_.rend());
        }  else if (conn_type == PathEngine::HEAD_HEAD) {
            if (road_it == result.begin()) {
                coord_list.insert(coord_list.end(), (*road_it)->points_.rbegin(), (*road_it)->points_.rend());
            }
            coord_list.insert(coord_list.end(), ++((*next_it)->points_.begin()), (*next_it)->points_.end());
        } else if (conn_type == PathEngine::HEAD_TAIL) {
            if (road_it == result.begin()) {
                coord_list.insert(coord_list.end(), (*road_it)->points_.rbegin(), (*road_it)->points_.rend());
            }
            coord_list.insert(coord_list.end(), ++((*next_it)->points_.rbegin()), (*next_it)->points_.rend());
        }

        road_it = next_it;
    }

    vector<shared_ptr<KDCoord>> dense_coord_list;
    double angle = geo::geo_util::calcAngle(136.232323, 39.0,
                                            136.232323, 39.1);
    for(size_t i = 0; i < coord_list.size() - 1; ++i) {
        auto coord1 = coord_list[i];
        auto coord2 = coord_list[i+1];
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

    IManager* mesh_manage_ = data_manager.base_data_manager_;
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
    cout<<"result: "<<result.size()<<endl;
    list<shared_ptr<KDRoad>> res_list;
    Viterbi viterbi;
    viterbi.Compute(stepList.step_list_, false, res_list, mesh_manage_);

    list<shared_ptr<KDRoad>> match_list;



    string output_path = GlobalCache::GetInstance()->out_path();
    ShpOutPut::OutPutLinkList(output_path + "/path1", result);
    ShpOutPut::OutPutLinkList(output_path + "/path2", res_list);

    return true;
}