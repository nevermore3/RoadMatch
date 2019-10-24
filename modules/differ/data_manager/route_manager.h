#ifndef HDMAP_DIFFER_FUSION_ROUTE_MANAGER_H
#define HDMAP_DIFFER_FUSION_ROUTE_MANAGER_H

#include <string>
#include <unordered_map>
#include <vector>
#include <memory>
#include <map>
#include "data_types.h"
#include "geos/indexStrtree.h"
using namespace std;
using namespace geos::index::strtree;
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

    shared_ptr<STRtree> strtree_;

    unordered_map<string, shared_ptr<Route>>routes_;

    unordered_map<string, shared_ptr<MeshObj>> meshs_;

protected:
    RouteManager() {
        strtree_ = make_shared<geos::index::strtree::STRtree>();
    }

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

    shared_ptr<LineString> line_;

    vector<shared_ptr<KDCoord>> points_;

    shared_ptr<KDRoadNode> start_point_;

    shared_ptr<KDRoadNode> end_point_;

    int64_t f_node_id_;

    int64_t t_node_id_;


    ~Route() {
        vector<shared_ptr<KDCoord>>().swap(points_);
    }

};


#endif //HDMAP_DIFFER_FUSION_ROUTE_MANAGER_H
