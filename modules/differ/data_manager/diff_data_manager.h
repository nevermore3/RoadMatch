//
// Created by liujian on 19-7-9.
//

#ifndef TD_DATA_DIFFER_DIFF_DATA_MANAGER_H
#define TD_DATA_DIFFER_DIFF_DATA_MANAGER_H

#include "data_manager.h"
#include "geos/indexQuadtree.h"
#include "geos/indexStrtree.h"
#include "geos/geom/LineString.h"

using namespace geos::index::quadtree;
using namespace geos::geom;

class DiffDataManager : public IManager{
public:
    static DiffDataManager *GetInstance() {
        static DiffDataManager instance_;
        return &instance_;
    }
    virtual shared_ptr<MeshObj> GetMesh(const string& mesh_id);

    virtual shared_ptr<KDRoad> GetRoad(string mesh_id, int32_t road_id);

    bool LoadData(const KDExtent& extent);
    bool LoadRoad(string file_name, shared_ptr<STRtree> strtree);
    bool LoadNode(string file_name);

    void BuildBoundry();
    void RemoveNonHighWay();

private:
    void StringToVector(string, shared_ptr<KDRoadNode>, shared_ptr<MeshObj>);

    void OutputRoad(const string &filename);
    void OutputNode(const string &filename);
protected:
    DiffDataManager() {
        quadtree_ = make_shared<geos::index::quadtree::Quadtree>();
        strtree_ = make_shared<geos::index::strtree::STRtree>();
    }

public:
    unordered_map<string, shared_ptr<MeshObj>> meshs_;

    unordered_map<int64_t, set<shared_ptr<KDRoadNode>>> code_node_;
    unordered_map<int64_t, set<shared_ptr<KDRoad>>> code_road_;

    //unordered_map<int64_t, shared_ptr<KDRoad>> node_road_;


    //所有加载的道路数据空间索引
    shared_ptr<Quadtree> quadtree_;
 //   shared_ptr<STRtree> strtree_;

    unordered_map<int32_t, string> mesh_id_map_;
    set<string> mesh_set_;


private:
    const double LONGLAT_RATIO = 3600000.0;
};

#endif //TD_DATA_DIFFER_DIFF_DATA_MANAGER_H
