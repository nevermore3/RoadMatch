#ifndef TD_DATA_DIFFER_DATA_MANAGER_H
#define TD_DATA_DIFFER_DATA_MANAGER_H


#include "data_types.h"

#include "geos/indexQuadtree.h"
#include "geos/indexStrtree.h"

class MeshObj;

class IManager {
public:
    virtual ~IManager() = default;

public:
    virtual bool LoadData(const KDExtent& extent) = 0;
    virtual shared_ptr<MeshObj> GetMesh(const string& mesh_id) = 0;

    virtual shared_ptr<KDRoad> GetRoad(string mesh_id, int32_t road_id) = 0;

public:
    shared_ptr<STRtree> strtree_;
};

class DataManager {
public:
    DataManager() = default;
    ~DataManager() = default;

public:
    bool LoadData();

public:
    IManager* base_data_manager_;
    IManager* diff_data_manager_;
    KDExtent extent_;
};



#endif //TD_DATA_DIFFER_DATA_MANAGER_H
