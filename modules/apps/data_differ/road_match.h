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

    void CloseRoute(shared_ptr<Route>route, list<shared_ptr<KDRoad>> &result);

    void DiffRoad(shared_ptr<Route>route);
    void DiffRoad2(shared_ptr<Route>route);


    void MatchAdd();

    void MatchDelete();

    void OutputRoad(const string &path);

    void Statistic();

    void DoDiff(shared_ptr<Route> route, list<shared_ptr<KDRoad>> &result);
private:
    // 按照mesh管理的
    unordered_map<string, shared_ptr<MeshObj>> meshs_;

    void DeleteRoad(shared_ptr<KDRoad> road);

    void AddRoad(shared_ptr<KDRoad>newRoad);

    bool CheckMatchRoad(vector<shared_ptr<KDCoord>> &road, shared_ptr<Route> route);
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
