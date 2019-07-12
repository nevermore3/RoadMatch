//
// Created by gaoyanhong on 2018/12/10.
//

#ifndef MATCH_CORE_MESH_CACHE_H
#define MATCH_CORE_MESH_CACHE_H


#include "geos/indexQuadtree.h"
#include "geos/indexStrtree.h"
#include "geos/geom/LineString.h"

using namespace geos::index::quadtree;
using namespace geos::geom;

#include "mesh/MeshGridObjectExt.hpp"

using namespace kd::autohdmap;

#include "data_types.h"
#include "data_manager.h"

#include <unordered_map>

class MeshObj {
public:
    MeshObj();

    ~MeshObj() {}

    bool LoadMesh(const string &mesh_id, const string &mesh_path, shared_ptr<STRtree> quadtree);

private:
    bool LoadRoad(string file_name, shared_ptr<STRtree> quadtree);

    bool LoadNode(string file_name);

    bool BuildTopReleation();

    bool LoadAdjNode(string file_name);

public:
    string mesh_id_;

    unordered_map<int32_t, shared_ptr<KDRoad>> roads_;
    unordered_map<int32_t, shared_ptr<KDRoadNode>> road_nodes_;
    unordered_map<int64_t, shared_ptr<KDRoadNode>> adj_nodes_;


private:
    const double LONGLAT_RATIO = 3600.0;
};

class MeshManage : public IManager {

public:
    static MeshManage *GetInstance() {
        static MeshManage instance_;
        return &instance_;
    }

    shared_ptr<MeshObj> GetMesh(const KDExtent &extent);

    shared_ptr<MeshObj> GetMesh(const string& mesh_id);

    shared_ptr<KDRoad> GetRoad(string mesh_id, int32_t road_id);

    bool LoadData(const KDExtent &extent);

    bool MeshOutOfRange(const string& mesh_name);

    //获取相邻meshid
    bool GetAdjMeshNameList (const unordered_map<int32_t, string>& mesh_id_map,
                             const string& mesh_name,
                             vector<string>& mesh_list);

protected:
    MeshManage() {
        quadtree_ = make_shared<geos::index::quadtree::Quadtree>();
        strtree_ = make_shared<geos::index::strtree::STRtree>();

    }

private:
    string GetMeshFullPath(const string &meshId);

    int32_t MeshName2MeshID(const string& meshid);

public:
    unordered_map<string, shared_ptr<MeshObj>> meshs_;

    //所有加载的道路数据空间索引
    shared_ptr<Quadtree> quadtree_;
    unordered_map<int32_t, string> mesh_id_map_;

    set<string> mesh_set_;

    shared_ptr<STRtree> strtree_;

private:
    const double LONGLAT_RATIO = 3600000.0;
};

#endif //MATCH_CORE_MESH_CACHE_H
