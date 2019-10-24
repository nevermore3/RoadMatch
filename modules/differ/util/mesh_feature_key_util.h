#ifndef ROAD_MATCH_SERVICE_MESH_FEATURE_KEY_UTIL_H
#define ROAD_MATCH_SERVICE_MESH_FEATURE_KEY_UTIL_H

#include "data_manager/data_types.h"

class MeshFeatureKeyUtil{
public:
    static std::string BuildKey(const std::shared_ptr<KDRoad> &road);

    static std::string BuildKey(const std::string &mesh_id, int64_t road_id);
};
#endif //ROAD_MATCH_SERVICE_MESH_FEATURE_KEY_UTIL_H
