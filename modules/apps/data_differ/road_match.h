//
// Created by ubuntu on 19-7-30.
//

#ifndef HDMAP_DIFFER_FUSION_ROAD_MATCH_H
#define HDMAP_DIFFER_FUSION_ROAD_MATCH_H

#include "data_manager/data_manager.h"

class Route;

class DataManager;

class RoadMatch
{
public:
    static RoadMatch *GetInstance() {
        static RoadMatch instance_;
        return &instance_;
    }

    bool MatchProcess();

    void MatchRoute(shared_ptr<Route>route);

//    void AddRoad(shared_ptr<KDRoad>newRoad, string meshID);
//
//    void DeleteRoad(shared_ptr<KDRoad>deleteRoad, string meshID);
//
//    void UpdateRoad(shared_ptr<KDRoad>updateRoad, string meshID);
//
//    void OutputRoad(const string &path);
//
//    void Statistic();
private:
    // 按照mesh管理的
    unordered_map<string, shared_ptr<MeshObj>> meshs_;
//    void PreProcess();
//
//    RoadMatch() = default;
//    void DiffRoad(const shared_ptr<KDRoad> &road, vector<shared_ptr<KDRoad>> &obj, IManager *dataManager);

};


#endif //HDMAP_DIFFER_FUSION_ROAD_MATCH_H
