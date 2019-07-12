#include "mesh_feature_key_util.h"

std::string MeshFeatureKeyUtil::BuildKey(const std::shared_ptr<KDRoad> &road) {
    return road->mesh_id_ + to_string(road->id_);
}

std::string MeshFeatureKeyUtil::BuildKey(const std::string &mesh_id, int64_t road_id) {
    return mesh_id + to_string(road_id);
}

