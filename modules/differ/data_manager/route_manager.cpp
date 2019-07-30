//
// Created by ubuntu on 19-7-30.
//

#include "route_manager.h"

#include "data_types.h"
#include "mesh_manager.h"

#include "glog/logging.h"
#include "glog/log_severity.h"
#include "global_cache.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <iostream>
#include <algorithm>
#include <shp/ShpData.hpp>
#include "diff_data_manager.h"
using namespace std;

int64_t RouteManager::route_id_ = 0;

bool RouteManager::IsVisit(shared_ptr<KDRoad> road)
{
    return flag_.find(GetRoadKey(road)) != flag_.end();
}

string RouteManager::GetRoadKey(shared_ptr<KDRoad> road)
{
    return road->mesh_id_ + to_string(road->id_);
}

void RouteManager::Visit(shared_ptr<KDRoad> road)
{
    flag_.insert(make_pair<string, bool>(GetRoadKey(road), true));
}

void RouteManager::ExtendFrom(shared_ptr<Route> route, shared_ptr<KDRoad> road)
{
    if (road == nullptr) {
        cout<<"ERROR road is nullptr"<<endl;
        return;
    }
    //遍历过(有环存在)或者名字不同
    if (IsVisit(road) || road->road_name_ != route->route_name_)
        return;

    //遍历
    Visit(road);
    route->roads_.insert(route->roads_.begin(), road);
    route->points_.insert(route->points_.begin(), road->points_.begin(), road->points_.end());
    route->num_of_roads_++;
    route->total_length_ += road->length_;

    //
    route->f_node_id_ = road->f_node_id_;

    int64_t  fromNode = road->f_node_id_;
    unordered_map<int32_t, shared_ptr<KDRoadNode>> roadNodes = meshs_[road->mesh_id_]->road_nodes_;
    shared_ptr<KDRoadNode>node = roadNodes[fromNode];

    // 多岔口或者死路
    if (node->from_roads_.size() > 2 || (node->boundary_ == 0 && node->from_roads_.size() == 1))
        return;

    shared_ptr<KDRoad>fromRoad;
    //边界情况
    if (node->boundary_ == 1) {
        shared_ptr<KDRoadNode>adjNode = meshs_[node->adj_mesh_id_]->road_nodes_[node->adj_id_];
        fromRoad = adjNode->from_roads_.front();
    } else {
        for (const auto &roadObj : node->from_roads_) {
            if (roadObj != road) {
                fromRoad = roadObj;
                break;
            }
        }
    }
    if (fromRoad == nullptr) {
        cout<<"Error fromRoad !!!"<<endl;
        return;
    }
    ExtendFrom(route, fromRoad);
}

void RouteManager::ExtendTo(shared_ptr<Route> route, shared_ptr<KDRoad> road)
{
    if (road == nullptr) {
        cout<<"Error : road is null !!!"<<endl;
        return;
    }
    if (IsVisit(road) || road->road_name_ != route->route_name_)
        return;

    //遍历
    Visit(road);
    route->roads_.push_back(road);
    route->points_.insert(route->points_.end(), road->points_.begin(), road->points_.end());
    route->num_of_roads_++;
    route->total_length_ += road->length_;
    //
    route->t_node_id_ = road->t_node_id_;

    int64_t  toNode = road->t_node_id_;
    unordered_map<int32_t, shared_ptr<KDRoadNode>> roadNodes = meshs_[road->mesh_id_]->road_nodes_;
    shared_ptr<KDRoadNode>node = roadNodes[toNode];

    // 多岔口或者死路
    if (node->to_roads_.size() > 2 || (node->boundary_ == 0 && node->to_roads_.size() == 1))
        return;

    shared_ptr<KDRoad>toRoad;
    //边界情况
    if (node->boundary_ == 1) {
        shared_ptr<KDRoadNode>adjNode = meshs_[node->adj_mesh_id_]->road_nodes_[node->adj_id_];
        toRoad = adjNode->to_roads_.front();
    } else {
        for (const auto &roadObj : node->to_roads_) {
            if (roadObj != road) {
                toRoad = roadObj;
                break;
            }
        }
    }
    if (toRoad == nullptr) {
        cout<<"Error toRoad!!!"<<endl;
        return;
    }
    ExtendTo(route, toRoad);
}

bool RouteManager::Init()
{
    for (const auto &mesh : meshs_) {
        for (const auto &road : mesh.second->roads_) {
            shared_ptr<KDRoad> roadObj = road.second;
            string name = roadObj->road_name_;

            if (IsVisit(roadObj)) {
                //cout << "road : " << roadObj->id_ << "  Has Visited" << endl;
                continue;
            }

            shared_ptr<Route> route = make_shared<Route>(name);
            route->id_ = RouteManager::route_id_;
            RouteManager::route_id_++;

            Visit(roadObj);
            route->roads_.push_back(roadObj);
            route->total_length_ = roadObj->length_;
            route->num_of_roads_ = 1;
            route->points_.insert(route->points_.begin(), roadObj->points_.begin(), roadObj->points_.end());
            route->f_node_id_ = roadObj->f_node_id_;
            route->t_node_id_ = roadObj->t_node_id_;
            // connect left
            {
                int64_t fromNode = roadObj->f_node_id_;
                unordered_map<int32_t, shared_ptr<KDRoadNode>> roadNodes = meshs_[roadObj->mesh_id_]->road_nodes_;
                shared_ptr<KDRoadNode> node = roadNodes[fromNode];

                if (node->from_roads_.size() == 2 || node->boundary_ == 1) {
                    shared_ptr<KDRoad> fromRoad;
                    if (node->boundary_ == 1) {
                        shared_ptr<KDRoadNode>adjNode = meshs_[node->adj_mesh_id_]->road_nodes_[node->adj_id_];
                        fromRoad = adjNode->from_roads_.front();
                    } else {
                        for (const auto &froad : node->from_roads_) {
                            if (froad != roadObj) {
                                fromRoad = froad;
                                break;
                            }
                        }
                    }
                    if (fromRoad == nullptr) {
                        cout << "Error fromRoad !!!" << endl;
                    }
                    ExtendFrom(route, fromRoad);
                }
            }
            //connect right
            {
                int64_t toNode = roadObj->t_node_id_;
                unordered_map<int32_t, shared_ptr<KDRoadNode>> roadNodes = meshs_[roadObj->mesh_id_]->road_nodes_;
                shared_ptr<KDRoadNode> node = roadNodes[toNode];
                // 边界
                if (node->to_roads_.size() == 2 || node->boundary_ == 1) {
                    shared_ptr<KDRoad> toRoad;
                    if (node->boundary_ == 1) {
                        shared_ptr<KDRoadNode>adjNode = meshs_[node->adj_mesh_id_]->road_nodes_[node->adj_id_];
                        toRoad = adjNode->to_roads_.front();
                    } else {
                        for (const auto &troad : node->to_roads_) {
                            if (troad != roadObj) {
                                toRoad = troad;
                                break;
                            }
                        }
                    }
                    if (toRoad == nullptr) {
                        cout<<"Error ToRoad !!!"<<endl;
                    }
                    ExtendTo(route, toRoad);
                }
            }
            route->key_name_ = name + to_string(route->total_length_);
            if (routes_.find(route->key_name_) != routes_.end()) {
                cout<<"ERROR !!! key name "<<endl;
                return  false;
            }
            routes_.insert(make_pair(route->key_name_, route));
        }
    }
    return true;
}

void RouteManager::LoadRoute()
{
    DiffDataManager *diffDataManager = DiffDataManager::GetInstance();
    SetMesh(diffDataManager->meshs_);

    Init();
    string output = GlobalCache::GetInstance()->out_path();
    OutputRoad(output);
    OutputNode(output);
}


void RouteManager::OutputRoad(const string &filename)
{
    string outputPath = filename + "/route";
    if (access(outputPath.c_str(), F_OK) == -1) {
        if (mkdir(outputPath.c_str(), 0755) == -1) {
            LOG(ERROR)<<"mkdir output error !!!";
            return;
        }
    }
    string dbfName = outputPath + "/route_road.dbf";
    string shpName = outputPath + "/route_road.shp";

    SHPHandle ptrRoadShp_ = nullptr;
    DBFHandle ptrRoadDbf_ = nullptr;

    ptrRoadShp_ = SHPCreate(shpName.c_str(), SHPT_ARCZ);
    ptrRoadDbf_ = DBFCreate(dbfName.c_str());

    DBFAddField(ptrRoadDbf_, "ID", FTLong, 16, 0);
    DBFAddField(ptrRoadDbf_, "from_node", FTLong, 16, 0);
    DBFAddField(ptrRoadDbf_, "to_node", FTLong, 16, 0);
    DBFAddField(ptrRoadDbf_, "name", FTString, 64, 0);
    DBFAddField(ptrRoadDbf_, "length", FTDouble, 8, 2);


    int nCount = 0;
    for (auto route : routes_) {
        shared_ptr<Route>routeObj = route.second;
        size_t  coord_nums = routeObj->points_.size();
        if (coord_nums == 0) {
            LOG(ERROR)<<" coord nums is  0";
            continue;
        }
        double *coords_x = new double[coord_nums];
        double *coords_y = new double[coord_nums];
        double *coords_z = new double[coord_nums];

        for (int j = 0; j < coord_nums; j++) {
            coords_x[j] = routeObj->points_[j]->lng_;
            coords_y[j] = routeObj->points_[j]->lat_;
            coords_z[j] = routeObj->points_[j]->z_;
        }
        SHPObject *shpObj = SHPCreateSimpleObject(SHPT_ARCZ, (int) coord_nums, coords_x, coords_y, coords_z);
        nCount = SHPWriteObject(ptrRoadShp_, -1, shpObj);
        delete[] coords_x;
        delete[] coords_y;
        delete[] coords_z;

        DBFWriteLongAttribute(ptrRoadDbf_, nCount, 0, routeObj->id_);
        DBFWriteLongAttribute(ptrRoadDbf_, nCount, 1, routeObj->f_node_id_);
        DBFWriteLongAttribute(ptrRoadDbf_, nCount, 2, routeObj->t_node_id_);
        DBFWriteStringAttribute(ptrRoadDbf_, nCount, 3, routeObj->route_name_.c_str());
        DBFWriteDoubleAttribute(ptrRoadDbf_, nCount, 4, routeObj->total_length_);

        SHPDestroyObject(shpObj);

    }

    if (ptrRoadShp_ != nullptr) {
        SHPClose(ptrRoadShp_);
    }
    if (ptrRoadDbf_ != nullptr) {
        DBFClose(ptrRoadDbf_);
    }
}

void RouteManager::OutputNode(const string &filename)
{
    string outputPath = filename + "/route";
    if (access(outputPath.c_str(), F_OK) == -1) {
        if (mkdir(outputPath.c_str(), 0755) == -1) {
            LOG(ERROR)<<"mkdir output error !!!";
            return;
        }
    }
    string dbfName = outputPath + "/route_node.dbf";
    string shpName = outputPath + "/route_node.shp";

    SHPHandle ptrRoadShp_ = nullptr;
    DBFHandle ptrRoadDbf_ = nullptr;

    ptrRoadShp_ = SHPCreate(shpName.c_str(), SHPT_POINTZ);
    ptrRoadDbf_ = DBFCreate(dbfName.c_str());

    DBFAddField(ptrRoadDbf_, "ID", FTLong, 16, 0);

    int nCount = 0;

    for (auto mesh : meshs_) {
        shared_ptr<MeshObj> meshobj = mesh.second;

        for (auto i: meshobj->road_nodes_) {
            shared_ptr<KDRoadNode> nodeobj = i.second;

            int coord_nums = 1;
            double *coords_x = new double[coord_nums];
            double *coords_y = new double[coord_nums];
            double *coords_z = new double[coord_nums];

            for (int j = 0; j < coord_nums; j++) {
                coords_x[j] = nodeobj->coord_.lng_;
                coords_y[j] = nodeobj->coord_.lat_;
                coords_z[j] = nodeobj->coord_.z_;
            }

            SHPObject *shpObj = SHPCreateSimpleObject(SHPT_POINTZ, 1, coords_x, coords_y, coords_z);
            nCount = SHPWriteObject(ptrRoadShp_, -1, shpObj);
            delete[] coords_x;
            delete[] coords_y;
            delete[] coords_z;

            DBFWriteLongAttribute(ptrRoadDbf_, nCount, 0, nodeobj->id_);
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