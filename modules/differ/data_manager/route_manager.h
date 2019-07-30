//
// Created by ubuntu on 19-7-30.
//

#ifndef HDMAP_DIFFER_FUSION_ROUTE_MANAGER_H
#define HDMAP_DIFFER_FUSION_ROUTE_MANAGER_H

#include <string>
#include <unordered_map>
#include <vector>
#include <memory>
#include <map>
#include "data_types.h"
using namespace std;

class MeshObj;
class Route;

class RouteManager
{
public:
    static RouteManager *GetInstance() {
        static RouteManager instance_;
        return &instance_;
    }

    static int64_t route_id_;

    bool Init();

    void LoadRoute();

    void SetMesh(unordered_map<string, shared_ptr<MeshObj>> mesh) {
        meshs_ = std::move(mesh);
    }

    unordered_map<string, shared_ptr<Route>>routes_;

    unordered_map<string, shared_ptr<MeshObj>> meshs_;

protected:


private:
    void OutputRoad(const string &filename);
    void OutputNode(const string &filename);

    void ExtendFrom(shared_ptr<Route>route, shared_ptr<KDRoad>road);

    void ExtendTo(shared_ptr<Route>route, shared_ptr<KDRoad>road);

    bool IsVisit(shared_ptr<KDRoad> road);

    string GetRoadKey(shared_ptr<KDRoad>road);

    void Visit(shared_ptr<KDRoad>road);

    map<string, bool>flag_;

};




class Route
{
public:
    explicit Route(string roadName) : route_name_(std::move(roadName)) {}
    int64_t id_;

    string key_name_;

    string route_name_;

    size_t num_of_roads_;

    vector<shared_ptr<KDRoad>>roads_;

    double total_length_;

    vector<shared_ptr<KDCoord>> points_;

    int64_t f_node_id_;

    int64_t t_node_id_;


    ~Route() {
        vector<shared_ptr<KDCoord>>().swap(points_);
    }

};


#endif //HDMAP_DIFFER_FUSION_ROUTE_MANAGER_H
