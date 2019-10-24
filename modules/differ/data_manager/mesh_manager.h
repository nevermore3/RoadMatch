#ifndef MATCH_CORE_MESH_CACHE_H
#define MATCH_CORE_MESH_CACHE_H

#include "data_types.h"
#include "data_manager.h"

#include "geos/geom/LineString.h"
#include "mesh/MeshGridObjectExt.hpp"

#include <unordered_map>


using namespace kd::autohdmap;
using namespace geos::index::quadtree;
using namespace geos::geom;
// 管理一个mesh
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

    // 保存road  key: roadID, value: road对象
    unordered_map<int32_t, shared_ptr<KDRoad>> roads_;

    // 保存节点  key: nodeID value: node对象
    unordered_map<int32_t, shared_ptr<KDRoadNode>> road_nodes_;

    // 保存当前mesh中临结到节点  key: mortoncode  value :node对象
    unordered_map<int64_t, shared_ptr<KDRoadNode>> adj_nodes_;


private:
    const double LONGLAT_RATIO = 3600.0;
};

class MeshManager : public IManager {

public:
    static MeshManager *GetInstance() {
        static MeshManager instance_;
        return &instance_;
    }

    shared_ptr<MeshObj> GetMesh(const KDExtent &extent);

    shared_ptr<MeshObj> GetMesh(const string& mesh_id);

    shared_ptr<KDRoad> GetRoad(string mesh_id, int32_t road_id);

    bool LoadData(const KDExtent &extent);

protected:
    MeshManager() {
        quadtree_ = make_shared<geos::index::quadtree::Quadtree>();
        strtree_ = make_shared<geos::index::strtree::STRtree>();

    }

private:
    string GetMeshFullPath(const string &meshId);

    int32_t MeshName2MeshID(const string& meshid);

    bool MeshOutOfRange(const string& mesh_name);

    //获取相邻meshid
    bool GetAdjMeshNameList (const unordered_map<int32_t, string>& mesh_id_map,
                             const string& mesh_name,
                             vector<string>& mesh_list);

public:
    //key : ""J50F009012"" value: mesh对象
    unordered_map<string, shared_ptr<MeshObj>> meshs_;

    //所有加载的道路数据空间索引
    shared_ptr<Quadtree> quadtree_;
    //key : 根据value生成到数字  value: "J50F009012"
    unordered_map<int32_t, string> mesh_id_map_;

    set<string> mesh_set_;

private:
    friend class MeshObj;
    const double LONGLAT_RATIO = 3600000.0;
};

#endif //MATCH_CORE_MESH_CACHE_H
