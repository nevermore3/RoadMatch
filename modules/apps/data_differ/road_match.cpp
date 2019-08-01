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
#include "road_sort.h"

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
    for (const auto &route : routeManager->routes_) {
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
            for (auto road : routeObj->roads_) {
                AddRoad(road);
            }
        } else {
            //if (routeObj->id_ == 370)
                DiffRoad(routeObj);
        }
    }
    
    Statistic();
    GlobalCache *globalCache = GlobalCache::GetInstance();
    string output = globalCache->out_path();
    OutputRoad(output);
    return true;
}

bool RoadMatch::CheckMatchRoad(vector<shared_ptr<KDCoord>> &road, shared_ptr<Route> baseRoute)
{
    // 先检查是否route 太短 造成road两端都无法匹配到route
    size_t  count = 0;   //匹配上的形点个数
    size_t  totalCount = 0;  //总的形点个数
    int8_t locate = 0;
    double distance = 0;

    for (size_t i = 0; i < baseRoute->points_.size(); i++) {
        distance = Distance::distance(baseRoute->points_[i], road, nullptr, nullptr, &locate, 0, -1) / 100;
        if (locate == 0 && distance < 20) {
            count++;
        }
        totalCount++;
    }
    double t = static_cast<double>(count) / totalCount;
    if (t > 0.8)
        return true;

    RouteManager *routeManager = RouteManager::GetInstance();

    shared_ptr<KDCoord>startPoint = road.front();
    shared_ptr<KDCoord>endPoint = road.back();
    vector<shared_ptr<QueryRoute>>startRoutes;
    vector<shared_ptr<QueryRoute>>endRoutes;

    double bufferSize = 20;
    // 起始点最近的3条route
    {
        shared_ptr<Point> point = GeometryUtil::CreatePoint(startPoint);
        shared_ptr<geos::geom::Geometry> geom_buffer(point->buffer(bufferSize));
        //query
        vector<void *> queryObjs;
        routeManager->strtree_->query(geom_buffer->getEnvelopeInternal(), queryObjs);
        for (auto i : queryObjs) {
            auto *route = static_cast<Route *>(i);
            shared_ptr<Route> routeObj = routeManager->routes_[route->key_name_];
            double dis = Distance::distance(startPoint, route->points_, nullptr, nullptr, nullptr, 0, -1) / 100;

            shared_ptr<QueryRoute> queryRoute (new QueryRoute(routeObj));
            queryRoute->start_distance_ = dis;

            startRoutes.push_back(queryRoute);
        }
        FilterRoad(startRoutes, 0);
    }

    // 终止点最近的3条route
    {
        shared_ptr<Point> point = GeometryUtil::CreatePoint(endPoint);
        shared_ptr<geos::geom::Geometry> geom_buffer(point->buffer(bufferSize));
        //query
        vector<void *> queryObjs;
        routeManager->strtree_->query(geom_buffer->getEnvelopeInternal(), queryObjs);
        for (auto i : queryObjs) {
            auto *route = static_cast<Route *>(i);
            shared_ptr<Route> routeObj = routeManager->routes_[route->key_name_];
            double dis = Distance::distance(endPoint, route->points_, nullptr, nullptr, nullptr, 0, -1) / 100;
            shared_ptr<QueryRoute> queryRoad (new QueryRoute(routeObj));
            queryRoad->end_distance_ = dis;
            endRoutes.push_back(queryRoad);
        }
        FilterRoad(endRoutes, 1);
    }

    // 判断是否是同一条route
    if (!startRoutes.empty() && !endRoutes.empty()) {
        shared_ptr<Route>sRoute = startRoutes.front()->route_;
        shared_ptr<Route>eRoute = endRoutes.front()->route_;
        //都没有匹配同一条route，则认为是新增
        if (sRoute != baseRoute && eRoute != baseRoute) {
            for (auto roadObj : baseRoute->roads_) {
                AddRoad(roadObj);
            }
            return true;
        }
    }
    return false;

}

void RoadMatch::DoDiff(shared_ptr<Route> route, list<shared_ptr<KDRoad>> &result)
{
    vector<shared_ptr<Points>>matchRoute;

    for (size_t i = 0; i < route->num_of_roads_; i++) {
        shared_ptr<KDRoad>road = route->roads_[i];
        if (i == route->num_of_roads_ - 1) {
            for (size_t j = 0; j < road->points_.size(); j++) {
                shared_ptr<Points>newPoint = make_shared<Points>(road->points_[j], road);
                matchRoute.push_back(newPoint);
            }
        } else {
            for (size_t j = 0; j < road->points_.size() - 1; j++) {
                shared_ptr<Points>newPoint = make_shared<Points>(road->points_[j], road);
                matchRoute.push_back(newPoint);
            }
        }
    }
    if (matchRoute.size() != route->points_.size()) {
        cout<<"ERROR !!!!!"<<endl;
        assert(0);
    }

    //合成一条route
    vector<shared_ptr<KDCoord>>matchRoad;
    auto iter = result.begin();
    while (iter != result.end()) {
        auto iter2 = iter;
        if (++iter2 == result.end()) {
            matchRoad.insert(matchRoad.end(), (*iter)->points_.begin(), (*iter)->points_.end());
        } else {
            matchRoad.insert(matchRoad.end(), (*iter)->points_.begin(), (*iter)->points_.end() - 1);
        }
        iter++;
    }

//    if (CheckMatchRoad(matchRoad, route)) {
//        return;
//    }

    // 两条route之间的比较
    double threshold = 20;
    double distance = 0;
    int8_t  locate = 0;
    int32_t posIndex = 0;
    shared_ptr<KDCoord> foot = make_shared<KDCoord>();
    size_t count = 0;
    double tempLength = 0;

    shared_ptr<KDCoord>pre = nullptr;
    shared_ptr<KDCoord>current = nullptr;
    vector<shared_ptr<KDCoord>>newPoints;

    size_t  i = 0;

    for (; i < route->points_.size(); i++) {
        if (i != 0) {
            pre = current;
        }
        current = route->points_[i];
        distance = Distance::distance(route->points_[i], matchRoad, foot, &posIndex, &locate, 0, -1) / 100;
        if ((locate == 0  && distance < threshold) ) {
            if (count != 0) {
                if (count >=2 && tempLength > 20) {
                    shared_ptr<KDRoad>newRoad  = make_shared<KDRoad>();
                    newRoad->points_.swap(newPoints);
                    newRoad->length_ = tempLength;
                    newRoad->id_ = matchRoute[i]->road_->id_;
                    newRoad->mesh_id_ = matchRoute[i]->road_->mesh_id_;
                    AddRoad(newRoad);
                } else {
                    //tempLength太小的忽略不记为新增
                    vector<shared_ptr<KDCoord>>().swap(newPoints);
                }
                count = 0;
                tempLength = 0;
            }
            continue;
        }

        if (locate != 0 || distance > threshold) {
            if (count != 0) {
                tempLength += Distance::distance(pre, current) / 100;
            }
            count++;
            newPoints.push_back(route->points_[i]);
        }

    }

    if (count >= 0  &&  tempLength > 20) {
        shared_ptr<KDRoad>newRoad  = make_shared<KDRoad>();
        newRoad->points_.swap(newPoints);
        newRoad->length_ = tempLength;
        newRoad->id_ = matchRoute[i - 1]->road_->id_;
        newRoad->mesh_id_ = matchRoute[i - 1]->road_->mesh_id_;
        AddRoad(newRoad);
    }

}

void RoadMatch::DiffRoad(shared_ptr<Route> route)
{
    list<shared_ptr<KDRoad>> result;
    CloseRoute(route, result);
    if (result.empty()) {
        for (const auto &road : route->roads_) {
            AddRoad(road);
        }
        return;
    }

    //若匹配到的roads的总长度和route的长度 远远不成比例则认为新增
//    double distance = 0;
//    for (const auto &road : result) {
//        distance += road->length_;
//    }
//    double value1 = abs(distance - route->total_length_) / distance;
//    double value2 = abs(distance - route->total_length_) / route->total_length_;

//    if (value1 > 6 || value2 > 6) {
//        for (const auto &road : route->roads_) {
//            AddRoad(road);
//        }
//    } else {
//        DoDiff(route, result);
//    }
    // 如果result.size == 1 可以通过角度判断是否新增
    if (result.size() == 1) {
        // 计算两条route的角度
        //TODO
        if (CheckMatchRoad(result.front()->points_, route))
            return;
    }

    DoDiff(route, result);
}

void RoadMatch::DiffRoad2(shared_ptr<Route> route)
{
    MeshManager *baseManager = MeshManager::GetInstance();

    shared_ptr<KDRoadNode>startNode = route->start_point_;
    shared_ptr<KDRoadNode>endNode = route->end_point_;


    vector<shared_ptr<QueryRoad>>startRoads;
    vector<shared_ptr<QueryRoad>>endRoads;

    double bufferSize = 20;
    // 起始点最近的3条road
    {
        shared_ptr<Point> point = GeometryUtil::CreatePoint(&startNode->coord_);
        shared_ptr<geos::geom::Geometry> geom_buffer(point->buffer(bufferSize));
        //query
        vector<void *> queryObjs;
        baseManager->strtree_->query(geom_buffer->getEnvelopeInternal(), queryObjs);
        for (auto i : queryObjs) {
            auto *road = static_cast<KDRoad *>(i);
            shared_ptr<KDRoad> roadObj = baseManager->meshs_[road->mesh_id_]->roads_[road->id_];
            double dis = Distance::distance(route->points_[0], roadObj->points_, nullptr, nullptr, nullptr, 0, -1) / 100;
            shared_ptr<QueryRoad> queryRoad (new QueryRoad(roadObj));
            queryRoad->start_distance_ = dis;
            startRoads.push_back(queryRoad);
        }
        FilterRoad(startRoads, 0);
    }

    // 终止点最近的3条road
    {
        shared_ptr<Point> point = GeometryUtil::CreatePoint(&endNode->coord_);
        shared_ptr<geos::geom::Geometry> geom_buffer(point->buffer(bufferSize));
        //query
        vector<void *> queryObjs;
        baseManager->strtree_->query(geom_buffer->getEnvelopeInternal(), queryObjs);
        for (auto i : queryObjs) {
            auto *road = static_cast<KDRoad *>(i);
            shared_ptr<KDRoad> roadObj = baseManager->meshs_[road->mesh_id_]->roads_[road->id_];
            double dis = Distance::distance(route->points_[route->points_.size() - 1], roadObj->points_,
                                            nullptr, nullptr, nullptr, 0, -1) / 100;
            shared_ptr<QueryRoad> queryRoad (new QueryRoad(roadObj));
            queryRoad->end_distance_ = dis;
            endRoads.push_back(queryRoad);
        }
        FilterRoad(endRoads, 1);

    }

    PathEngine engine;
    engine.SetSearchCount(50);

    for (auto startRoad : startRoads) {
        for (auto endRoad : endRoads) {
            std::list<shared_ptr<KDRoad>> result;
            if (engine.FindPath(baseManager, -1, startRoad->road_, startRoad->road_->points_[0],
                                endRoad->road_, endRoad->road_->points_[0], result)) {
                //TODO
            }
        }
    }

}


void RoadMatch::CloseRoute(shared_ptr<Route> route, list<shared_ptr<KDRoad>> &result){
    PathEngine engine;
    engine.SetSearchCount(20);
//    if(route->id_ != 12)
//        return;
    vector<shared_ptr<KDCoord>> dense_coord_list;
//    double angle = geo::geo_util::calcAngle(136.232323, 39.0,
//                                            136.232323, 39.1);


    double ht_dist = Distance::distance(route->points_[0]->lng_, route->points_[0]->lat_,
                                        route->points_[route->points_.size() - 1]->lng_,
                                        route->points_[route->points_.size() - 1]->lat_) / 100;
    bool slip = false;
    if(ht_dist < 200.0 && route->total_length_ / ht_dist > 3.0)
        slip = true;

    for(size_t i = 0; i < route->points_.size() - 1; ++i) {
        auto coord1 = route->points_[i];
        auto coord2 = route->points_[i+1];
        if(i == 0)
            dense_coord_list.emplace_back(coord1);
        if(Distance::distance(coord1, coord2) < 5000) {
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
            coord->lng_ = coord1->lng_ + cnt * 50.0 *cos(angle)/110000;
            coord->lat_ = coord1->lat_ + cnt * 50.0 *sin(angle)/110000;
            dense_coord_list.emplace_back(coord);

            if(Distance::distance(coord, coord2) < 5000)
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
        shared_ptr<geos::geom::Geometry> geom_buffer(point->buffer(50.0));
        vector<void *> queryObjs;
        mesh_manage_->strtree_->query(geom_buffer->getEnvelopeInternal(), queryObjs);

        if (queryObjs.size() == 0) {
            continue;
        }

        double coord_angle = 0;
        if (index == 0) {
            coord_angle = geo::geo_util::calcAngle(coord->lng_, coord->lat_,
                                                   dense_coord_list[1]->lng_,
                                                   dense_coord_list[1]->lat_);
            coord_angle = coord_angle / M_PI * 180.0;
        } else {
            int start_index = index;
            while(start_index-- > 0) {
                if(Distance::distance(dense_coord_list[start_index], coord) > 1000) {
                    coord_angle = geo::geo_util::calcAngle(dense_coord_list[start_index]->lng_,
                                                           dense_coord_list[start_index]->lat_,
                                                           coord->lng_, coord->lat_);
                    coord_angle = coord_angle / M_PI * 180.0;
                    if(coord_angle > 180.0)
                        coord_angle = coord_angle - 360;
                    break;
                }
            }
        }

        //构建可能的候选项
        shared_ptr<CadidatesStep> step = make_shared<CadidatesStep>();
        bool valid = false;
        for (auto &record : queryObjs) {
            KDRoad *road = (KDRoad *) record;
            int pos_index;
            shared_ptr<KDCoord> foot = make_shared<KDCoord>();
            double distance = Distance::distance(coord, road->points_, foot, &pos_index);
            if (distance > 100.0 * 100)
                continue;

            double base_angle = 0;
            if(pos_index == road->points_.size() - 1) {
                base_angle = geo::geo_util::calcAngle(road->points_[pos_index - 1]->lng_,
                                                      road->points_[pos_index - 1]->lat_,
                                                      road->points_[pos_index]->lng_,
                                                      road->points_[pos_index]->lat_);
                base_angle = base_angle / M_PI * 180.0;
                if(base_angle > 180.0)
                    base_angle = base_angle - 360;
            } else {
                base_angle = geo::geo_util::calcAngle(road->points_[pos_index]->lng_,
                                                      road->points_[pos_index]->lat_,
                                                      road->points_[pos_index + 1]->lng_,
                                                      road->points_[pos_index + 1]->lat_);
                base_angle = base_angle / M_PI * 180.0;
                if(base_angle > 180.0)
                    base_angle = base_angle - 360;
            }

            double angle_diff = fabs(coord_angle - base_angle);

            if (angle_diff > 180) {
                angle_diff = 360 - angle_diff;
            }
            if(angle_diff > 45.0)
                continue;

            int8_t start_loc;
            double start_dist2track = Distance::distance(road->points_[0],
                    route->points_, nullptr, nullptr, &start_loc);
            int8_t end_loc;
            double end_dist2track = Distance::distance(road->points_[road->points_.size() - 1],
                    route->points_, nullptr, nullptr, &end_loc);

            if (start_loc == 0) {
                if(start_dist2track > 10000.0)
                    continue;
            }

            if (end_loc == 0) {
                if(end_dist2track > 10000.0)
                    continue;
            }


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
    cout<<"route' size : "<<route->roads_.size()<<endl;
    //list<shared_ptr<KDRoad>> res_list;
    Viterbi viterbi;
    viterbi.Compute(stepList.step_list_, false, result, mesh_manage_, slip);

    string output_path = GlobalCache::GetInstance()->out_path();
    string outputPath = output_path + "/test";

    if (access(outputPath.c_str(), F_OK) == -1) {
        if (mkdir(outputPath.c_str(), 0755) == -1) {
            LOG(ERROR)<<"mkdir output error !!!";
            return;
        }
    }
    //ShpOutPut::OutPutLinkList(outputPath + "/path1", result);
    ShpOutPut::OutPutLinkList(outputPath + "/path2", result);
}



void RoadMatch::AddRoad(shared_ptr<KDRoad> newRoad)
{
    shared_ptr<MeshObj>meshObj;
    if (meshs_.find(newRoad->mesh_id_) != meshs_.end()) {
        meshObj = meshs_[newRoad->mesh_id_];
    } else {
        meshObj = make_shared<MeshObj>();
    }
    meshObj->roads_.insert(make_pair(newRoad->id_, newRoad));
    if (meshs_.find(newRoad->mesh_id_) == meshs_.end()) {
        meshs_.insert(make_pair(newRoad->mesh_id_, meshObj));
    }
}


void RoadMatch::OutputRoad(const string &path)
{
    string filename = path + "/add";

    if (access(filename.c_str(), F_OK) == -1) {
        if (mkdir(filename.c_str(), 0755) == -1) {
            LOG(ERROR)<<"mkdir add error !!!";
            return;
        }
    }
    string dbfName = filename + "/add_road.dbf";
    string shpName = filename + "/add_road.shp";

    SHPHandle ptrRoadShp_ = nullptr;
    DBFHandle ptrRoadDbf_ = nullptr;

    ptrRoadShp_ = SHPCreate(shpName.c_str(), SHPT_ARCZ);
    ptrRoadDbf_ = DBFCreate(dbfName.c_str());

    DBFAddField(ptrRoadDbf_, "ID", FTLong, 16, 0);
    DBFAddField(ptrRoadDbf_, "from_node", FTLong, 16, 0);
    DBFAddField(ptrRoadDbf_, "to_node", FTLong, 16, 0);
    DBFAddField(ptrRoadDbf_, "name", FTString, 64, 0);
    DBFAddField(ptrRoadDbf_, "length", FTDouble, 8, 2);
    DBFAddField(ptrRoadDbf_, "meshId", FTString, 64, 0);


    int nCount = 0;
    for (auto mesh : meshs_) {
        shared_ptr<MeshObj> meshobj = mesh.second;
        for (auto i: meshobj->roads_) {

            shared_ptr<KDRoad>road = i.second;
            size_t coord_nums = road->points_.size();
            if (coord_nums == 0) {
                LOG(ERROR)<<" coord nums is  0";
                continue;
            }
            double *coords_x = new double[coord_nums];
            double *coords_y = new double[coord_nums];
            double *coords_z = new double[coord_nums];

            for (int j = 0; j < coord_nums; j++) {
                coords_x[j] = road->points_[j]->lng_;
                coords_y[j] = road->points_[j]->lat_;
                coords_z[j] = road->points_[j]->z_;
            }

            SHPObject *shpObj = SHPCreateSimpleObject(SHPT_ARCZ, (int) coord_nums, coords_x, coords_y, coords_z);
            nCount = SHPWriteObject(ptrRoadShp_, -1, shpObj);
            delete[] coords_x;
            delete[] coords_y;
            delete[] coords_z;

            DBFWriteLongAttribute(ptrRoadDbf_, nCount, 0, road->id_);
            DBFWriteLongAttribute(ptrRoadDbf_, nCount, 1, road->f_node_id_);
            DBFWriteLongAttribute(ptrRoadDbf_, nCount, 2, road->t_node_id_);
            DBFWriteStringAttribute(ptrRoadDbf_, nCount, 3, road->road_name_.c_str());
            DBFWriteDoubleAttribute(ptrRoadDbf_, nCount, 4, road->length_);
            DBFWriteStringAttribute(ptrRoadDbf_, nCount, 5, road->mesh_id_.c_str());
            SHPDestroyObject(shpObj);
        }
    }
    if (ptrRoadShp_ != nullptr) {
        SHPClose(ptrRoadShp_);
    }
    if (ptrRoadDbf_ != nullptr) {
        DBFClose(ptrRoadDbf_);
    }

}

void RoadMatch::Statistic()
{
    // mesh中road条数和路径总长度
    MeshManager *meshManager = MeshManager::GetInstance();
    double numOfMeshRoad = 0;
    double lengthOfMeshRoad = 0;
    for (auto mesh : meshManager->meshs_) {
        shared_ptr<MeshObj>meshObj = mesh.second;
        unordered_map<int32_t, shared_ptr<KDRoad>> roads = meshObj->roads_;
        numOfMeshRoad += roads.size();
        for (auto road : roads) {
            lengthOfMeshRoad += road.second->length_;
        }
    }

    // diff中road条数和路径总长度
    DiffDataManager *diffDataManager = DiffDataManager::GetInstance();
    double numOfDiffRoad = 0;
    double lengthOfDiffRoad = 0;
    for (auto mesh : diffDataManager->meshs_) {
        shared_ptr<MeshObj>meshObj = mesh.second;
        unordered_map<int32_t , shared_ptr<KDRoad>> roads = meshObj->roads_;
        numOfDiffRoad += roads.size();
        for (auto road : roads) {
            lengthOfDiffRoad += road.second->length_;
        }
    }

    // 新增的路径条数和路径总长度
    double numOfAddRoad = 0;
    double lengthOfAddRoad = 0;
    for (auto mesh : meshs_) {
        shared_ptr<MeshObj>meshObj = mesh.second;
        unordered_map<int32_t, shared_ptr<KDRoad>> roads = meshObj->roads_;
        numOfAddRoad += roads.size();
        for (auto road : roads) {
            lengthOfAddRoad += road.second->length_;
        }
    }

    cout<<"基础路网路径条数  "<<"\t"<<numOfMeshRoad<<endl;
    cout<<"基础路网路径总长度"<<"\t"<<lengthOfMeshRoad<<endl;
    cout<<"差分路网路径条数  "<<"\t"<<numOfDiffRoad<<endl;
    cout<<"差分路网路径总长度"<<"\t"<<lengthOfDiffRoad<<endl;
    cout<<"新增路径条数     "<<"\t"<<numOfAddRoad<<endl;
    cout<<"新增路径总长度    "<<"\t"<<lengthOfAddRoad<<endl;

}