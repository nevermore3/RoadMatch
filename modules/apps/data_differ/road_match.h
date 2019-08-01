//
// Created by ubuntu on 19-7-30.
//

#ifndef HDMAP_DIFFER_FUSION_ROAD_MATCH_H
#define HDMAP_DIFFER_FUSION_ROAD_MATCH_H

#include "data_manager/data_manager.h"
#include "road_sort.h"
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

    void CloseRoute(shared_ptr<Route>route, list<shared_ptr<KDRoad>> &result);

    void AddRoad(shared_ptr<KDRoad>newRoad);

    void DiffRoad(shared_ptr<Route>route);
    void DiffRoad2(shared_ptr<Route>route);
//
//    void DeleteRoad(shared_ptr<KDRoad>deleteRoad, string meshID);
//
//    void UpdateRoad(shared_ptr<KDRoad>updateRoad, string meshID);
//
    void OutputRoad(const string &path);
//
    void Statistic();

    void DoDiff(shared_ptr<Route> route, list<shared_ptr<KDRoad>> &result);
private:
    // 按照mesh管理的
    unordered_map<string, shared_ptr<MeshObj>> meshs_;
//    void PreProcess();
//
//    RoadMatch() = default;
//    void DiffRoad(const shared_ptr<KDRoad> &road, vector<shared_ptr<KDRoad>> &obj, IManager *dataManager);
    void FilterRoad(vector<shared_ptr<QueryRoad>>array, int flag);

};


class Points
{
public:
    shared_ptr<KDCoord>point_;
    shared_ptr<KDRoad>road_;
    shared_ptr<KDRoad>matched_road_;
    shared_ptr<KDCoord>matched_point_;
    Points(shared_ptr<KDCoord>point, shared_ptr<KDRoad>road) : point_(point), road_(road) {}
};

#endif //HDMAP_DIFFER_FUSION_ROAD_MATCH_H
