//
// Created by gaoyanhong on 2018/12/10.
//

#include "mesh_manager.h"

#include "geos/geom/Coordinate.h"
#include "geos/geom/GeometryFactory.h"
#include "geos/geom/CoordinateSequence.h"
#include "geos/geom/CoordinateArraySequence.h"
#include "geos/geom/Point.h"

#include <shp/ShpData.hpp>

#include <mvg/Coordinates.hpp>

using namespace kd::automap;

#include "global_cache.h"

#include "util/match_geo_util.h"
#include "util/geometry_util.h"
#include "util/maccoord_offset.h"
#include "util/morton_code.h"

#include "glog/logging.h"
#include "glog/log_severity.h"

MeshObj::MeshObj() {

}

bool MeshObj::LoadMesh(const string &mesh_id, const string &mesh_path, shared_ptr<STRtree> quadtree) {

    //cout << "[Debug] Load mesh begin, mesh id " << mesh_id << endl;
    mesh_id_ = mesh_id;
    MeshManager * meshManage = MeshManager::GetInstance();
    vector<string> adj_mesh_list;
    if (meshManage->GetAdjMeshNameList(meshManage->mesh_id_map_,
                                       mesh_id, adj_mesh_list)) {
        for (auto adj_mesh_name : adj_mesh_list) {
            if (meshManage->MeshOutOfRange(adj_mesh_name))
                continue;
            string mesh_data_path = GlobalCache::GetInstance()->mesh_data_path();
            string path = mesh_data_path + "/" + adj_mesh_name + "/";
            string adjNodeFile = path + "Node";
            if (!LoadAdjNode(adjNodeFile)) {
                LOG(WARNING) << "Load adj " << adjNodeFile << " road error.";
            }
        }
    }

    //加载road,node，并建立之间的拓扑关系
    string roadFile = mesh_path + "Road";
    if (!LoadRoad(roadFile, quadtree)) {
        LOG(ERROR) << "Load road error.";
        return false;
    }

    string nodeFile = mesh_path + "Node";
    if (!LoadNode(nodeFile)) {
        LOG(ERROR) << "Load node error.";
        return false;
    }

    if (!BuildTopReleation()) {
        LOG(ERROR) << "Build top relation error.";
        return false;
    }

    return true;
}

bool MeshObj::LoadRoad(string file_name, shared_ptr<STRtree> quadtree) {

    ShpData shpData(file_name);
    if (!shpData.isInit()) {
        return false;
    }

    const geos::geom::GeometryFactory *gf = geos::geom::GeometryFactory::getDefaultInstance();

    int record_nums = shpData.getRecords();
    for (int i = 0; i < record_nums; i++) {
        SHPObject *shpObject = shpData.readShpObject(i);

        if (!shpObject || shpObject->nSHPType != SHPT_ARC)
            continue;

        //read road
        shared_ptr<KDRoad> road = make_shared<KDRoad>();

        //read att
        road->id_ = shpData.readIntField(i, "ROAD_ID");
        road->f_node_id_ = shpData.readIntField(i, "FNODE_");
        road->t_node_id_ = shpData.readIntField(i, "TNODE_");
        road->road_class_ = shpData.readIntField(i, "ROADCLASS");
        road->form_way_ = shpData.readIntField(i, "FORMWAY");
        road->mesh_id_ = shpData.readStringField(i, "MESH");
        road->length_ = shpData.readDoubleField(i, "LENGTH");
        road->direction_ = shpData.readIntField(i, "DIRECTION");
        road->direction_ = 1;
        if (road->road_class_ == 100 ) {
            //分类为100的数据为边框，需要过滤掉
            continue;
        }

//        if (road->road_class_ >= 47000 && road->road_class_ != 51000 && road->form_way_ != 2  && road->road_class_ != 52000)
//            continue;

//        if (road->road_class_ != 41000)
//            continue;

        //read road geometry
        int vertex_nums = shpObject->nVertices;
        road->points_.resize(vertex_nums);
        for (int j = 0; j < vertex_nums; j++) {
            road->points_[j] = make_shared<KDCoord>();

            mac_coord_deoffset(shpObject->padfX[j]/LONGLAT_RATIO, shpObject->padfY[j]/LONGLAT_RATIO,
                        &road->points_[j]->lng_, &road->points_[j]->lat_);

            road->points_[j]->z_ = shpObject->padfZ[j];
        }

        //build road geos object
        CoordinateSequence *cl = new CoordinateArraySequence();
        for (int j = 0; j < vertex_nums; j++) {

            double utmX, utmY;
            char zone[4] = {0};
            Coordinates::ll2utm(road->points_[j]->lat_, road->points_[j]->lng_, utmX, utmY, zone);

            cl->add(Coordinate(utmX, utmY, shpObject->padfZ[j]));
        }

        road->line_.reset(gf->createLineString(cl));

  //      quadtree->insert(road->line_->getEnvelopeInternal(), road.get());
        quadtree->insert(road->line_->getEnvelopeInternal(), road.get());
        //add to cache
        roads_.insert(make_pair(road->id_, road));
    }

    return true;
}


bool MeshObj::LoadNode(string file_name) {

    ShpData shpData(file_name);
    if (!shpData.isInit()) {
        return false;
    }

    int record_nums = shpData.getRecords();
    for (int i = 0; i < record_nums; i++) {
        SHPObject *shpObject = shpData.readShpObject(i);

        if (!shpObject || shpObject->nSHPType != SHPT_POINT)
            continue;

        shared_ptr<KDRoadNode> node = make_shared<KDRoadNode>();

        //read att
        node->id_ = shpData.readIntField(i, "NODE_ID");
        node->mesh_id_ = shpData.readStringField(i, "MESH");

        node->boundary_ = shpData.readIntField(i, "BOUNDARY");

        //read geometry
        if (shpObject->nVertices != 1) {
            continue;
        }

        mac_coord_deoffset(shpObject->padfX[0]/LONGLAT_RATIO, shpObject->padfY[0]/LONGLAT_RATIO,
                           &node->coord_.lng_, &node->coord_.lat_);
        node->coord_.z_ = shpObject->padfZ[0];
        if (node->boundary_ != 0) {
            int64_t morton_code = MortonCode::GetMortonCodeFromRAWCoord(node->coord_.lng_, node->coord_.lat_);
            auto adjnode = adj_nodes_.find(morton_code);
            if (adjnode != adj_nodes_.end()) {

                node->adj_mesh_id_ = adjnode->second->mesh_id_;
                node->adj_id_ = adjnode->second->id_;
            } else {
                node->adj_mesh_id_ = "";
                node->adj_id_ = -1;
            }
        } else {
            node->adj_mesh_id_ = "";
            node->adj_id_ = -1;
        }
        //add to cache
        road_nodes_.insert(make_pair(node->id_, node));
    }

    return true;
}

bool MeshObj::BuildTopReleation() {

    //build relation of road and node
    LOG(INFO) << "Build relation of road and node. ";
    for (auto roadit : roads_) {

        int32_t roadid = roadit.first;
        shared_ptr<KDRoad> road = roadit.second;

//        if (road->road_class_ >= 47000 && road->road_class_ != 51000 && road->form_way_ != 2 && road->road_class_ != 52000) {
// //       if (road->road_class_ > 47000 && road->road_class_ != 51000) {
//
//                //"41000：高速公路
//            //42000：国道
//            //43000：城市快速路
//            //44000：城市主干路
//            //45000：城市次干路
//            //47000：城市普通道路
//            //51000：省道
//            //52000：县道
//            //53000：乡道
//            //54000：县乡村内部道路
//            //49：小路"
//            //过滤掉城市普通道路，保留城市次干路以上级别
//            continue;
//        }

        //from node
        auto fnodeit = road_nodes_.find(road->f_node_id_);
        if (fnodeit != road_nodes_.end()) {
            shared_ptr<KDRoadNode> node = fnodeit->second;
            node->from_roads_.emplace_back(road);
            if(road->direction_ == 1 || road->direction_ == 0)
                node->to_roads_.emplace_back(road);
        } else {
            LOG(ERROR) << "Not find node " << road->f_node_id_;
            return false;
        }

        //to node
        auto tnodeit = road_nodes_.find(road->t_node_id_);
        if (tnodeit != road_nodes_.end()) {
            shared_ptr<KDRoadNode> node = tnodeit->second;
            node->to_roads_.emplace_back(road);
            node->from_roads_.emplace_back(road);
//            if(road->direction_ == 1 || road->direction_ == 0)
//                node->from_roads_.emplace_back(road);
        } else {
            LOG(ERROR) << "Not find node " << road->t_node_id_;
            return false;
        }

    }

    std::list<int32_t> del_nodeids;
    for (auto nodeit : road_nodes_) {
        if (nodeit.second->from_roads_.empty() && nodeit.second->to_roads_.empty()) {
            del_nodeids.emplace_back(nodeit.first);
        }
    }

    for (auto nodeid : del_nodeids) {
        road_nodes_.erase(nodeid);
    }

    return true;
}

bool MeshObj::LoadAdjNode(string file_name) {

    ShpData shpData(file_name);
    if (!shpData.isInit()) {
        return false;
    }

    int record_nums = shpData.getRecords();
    for (int i = 0; i < record_nums; i++) {
        SHPObject *shpObject = shpData.readShpObject(i);
        if (!shpObject || shpObject->nSHPType != SHPT_POINT) {
            shpData.destroyShpObject(shpObject);
            continue;
        }

        shared_ptr<KDRoadNode> node = make_shared<KDRoadNode>();

        node->id_ = shpData.readIntField(i, "NODE_ID");
        node->mesh_id_ = shpData.readStringField(i, "MESH");
        node->boundary_ = shpData.readIntField(i, "BOUNDARY");

        if (node->boundary_ == 0) {
            shpData.destroyShpObject(shpObject);
            continue;
        }

        mac_coord_deoffset(shpObject->padfX[0]/LONGLAT_RATIO, shpObject->padfY[0]/LONGLAT_RATIO,
                           &node->coord_.lng_, &node->coord_.lat_);

        int64_t morton_code = MortonCode::GetMortonCodeFromRAWCoord(node->coord_.lng_, node->coord_.lat_);
        adj_nodes_.insert(make_pair(morton_code, node));
        shpData.destroyShpObject(shpObject);
    }

    return true;
}


/////////////////////////////////////////////////////////////////////////////////////
// meshmanage
/////////////////////////////////////////////////////////////////////////////////////

void OutputHighWay(MeshManager * meshManage) {
    string output_path = GlobalCache::GetInstance()->out_path();

    string dbf_file = output_path + "/HIGHROAD.dbf";
    string shp_file = output_path + "/HIGHROAD.shp";

    SHPHandle ptrRoadShp_ = nullptr;
    DBFHandle ptrRoadDbf_ = nullptr;
    ptrRoadShp_ = SHPCreate(shp_file.c_str(), SHPT_ARCZ);
    ptrRoadDbf_ = DBFCreate(dbf_file.c_str());

    DBFAddField(ptrRoadDbf_, "ID", FTLong, 16, 0);
    DBFAddField(ptrRoadDbf_, "MESHID", FTString, 16, 0);
    DBFAddField(ptrRoadDbf_, "DIRECTION", FTInteger, 10, 0);
    DBFAddField(ptrRoadDbf_, "SNODE_ID", FTLong, 16, 0);
    DBFAddField(ptrRoadDbf_, "ENODE_ID", FTLong, 16, 0);

    int nCount = 0;
    for (auto mesh_it : meshManage->meshs_) {
        const string& meshid = mesh_it.first;
        auto& mesh_obj = mesh_it.second;

        for(auto road_it: mesh_obj->roads_) {
            //write geo object
            int64_t roadid = road_it.first;
            auto& road = road_it.second;
            size_t coord_nums = road->points_.size();
            double *coords_x = new double[coord_nums];
            double *coords_y = new double[coord_nums];
            double *coords_z = new double[coord_nums];
            //
            for (int j = 0; j < coord_nums; j++) {
                coords_x[j] = road->points_[j]->lng_;
                coords_y[j] = road->points_[j]->lat_;
                coords_z[j] = road->points_[j]->z_;
            }
            //
            SHPObject *shpObj = SHPCreateSimpleObject(SHPT_ARCZ, (int) coord_nums, coords_x, coords_y, coords_z);
            nCount = SHPWriteObject(ptrRoadShp_, -1, shpObj);
            delete[] coords_x;
            delete[] coords_y;
            delete[] coords_z;

            DBFWriteLongAttribute(ptrRoadDbf_, nCount, 0, road->id_);
            DBFWriteStringAttribute(ptrRoadDbf_, nCount, 1, road->mesh_id_.c_str());
            DBFWriteIntegerAttribute(ptrRoadDbf_, nCount, 2, road->direction_);
            DBFWriteLongAttribute(ptrRoadDbf_, nCount, 3, road->f_node_id_);
            DBFWriteLongAttribute(ptrRoadDbf_, nCount, 4, road->t_node_id_);


            SHPDestroyObject(shpObj);
        }
    }

    if (ptrRoadShp_ != nullptr) {
        SHPClose(ptrRoadShp_);
    }
    if (ptrRoadDbf_ != nullptr) {
        DBFClose(ptrRoadDbf_);
    }
}

void OutputHighNode(MeshManager * meshManage) {
    string output_path = GlobalCache::GetInstance()->out_path();

    string dbf_file = output_path + "/HIGHNODE.dbf";
    string shp_file = output_path + "/HIGHNODE.shp";

    SHPHandle ptrRoadShp_ = nullptr;
    DBFHandle ptrRoadDbf_ = nullptr;
    ptrRoadShp_ = SHPCreate(shp_file.c_str(), SHPT_POINTZ);
    ptrRoadDbf_ = DBFCreate(dbf_file.c_str());

    DBFAddField(ptrRoadDbf_, "ID", FTLong, 16, 0);
    DBFAddField(ptrRoadDbf_, "MESHID", FTString, 16, 0);

    int nCount = 0;
    for (auto mesh_it : meshManage->meshs_) {
        const string& meshid = mesh_it.first;
        auto& mesh_obj = mesh_it.second;
        set<int32_t> id_set;
        for(auto road_it: mesh_obj->roads_) {
            //write geo object
            int32_t roadid = road_it.first;
            auto& road = road_it.second;

            int32_t f_node_id = road->f_node_id_;
            int32_t t_node_id = road->t_node_id_;


            if(id_set.find(f_node_id) == id_set.end()) {
                auto &f_node = mesh_obj->road_nodes_.find(f_node_id)->second;

                size_t coord_nums = 1;
                double *coords_x = new double[coord_nums];
                double *coords_y = new double[coord_nums];
                double *coords_z = new double[coord_nums];
                //
                coords_x[0] = f_node->coord_.lng_;
                coords_y[0] = f_node->coord_.lat_;
                coords_z[0] = f_node->coord_.z_;

                //
                SHPObject *shpObj = SHPCreateSimpleObject(SHPT_POINTZ, (int) coord_nums, coords_x, coords_y, coords_z);
                nCount = SHPWriteObject(ptrRoadShp_, -1, shpObj);
                delete[] coords_x;
                delete[] coords_y;
                delete[] coords_z;

                DBFWriteLongAttribute(ptrRoadDbf_, nCount, 0, f_node->id_);
                DBFWriteStringAttribute(ptrRoadDbf_, nCount, 1, f_node->mesh_id_.c_str());

                id_set.insert(f_node_id);

                SHPDestroyObject(shpObj);
            }

            if(id_set.find(t_node_id) == id_set.end()) {
                auto &t_node = mesh_obj->road_nodes_.find(t_node_id)->second;

                size_t coord_nums = 1;
                double *coords_x = new double[coord_nums];
                double *coords_y = new double[coord_nums];
                double *coords_z = new double[coord_nums];
                //
                coords_x[0] = t_node->coord_.lng_;
                coords_y[0] = t_node->coord_.lat_;
                coords_z[0] = t_node->coord_.z_;

                //
                SHPObject *shpObj = SHPCreateSimpleObject(SHPT_POINTZ, (int) coord_nums, coords_x, coords_y, coords_z);
                nCount = SHPWriteObject(ptrRoadShp_, -1, shpObj);
                delete[] coords_x;
                delete[] coords_y;
                delete[] coords_z;

                DBFWriteLongAttribute(ptrRoadDbf_, nCount, 0, t_node->id_);
                DBFWriteStringAttribute(ptrRoadDbf_, nCount, 1, t_node->mesh_id_.c_str());

                id_set.insert(t_node_id);

                SHPDestroyObject(shpObj);
            }

        }
    }

    if (ptrRoadShp_ != nullptr) {
        SHPClose(ptrRoadShp_);
    }
    if (ptrRoadDbf_ != nullptr) {
        DBFClose(ptrRoadDbf_);
    }
}

bool MeshManager::LoadData(const KDExtent &extent) {
    int xmin = (int) (extent.xmin_ * LONGLAT_RATIO);
    int xmax = (int) (extent.xmax_ * LONGLAT_RATIO);
    int ymin = (int) (extent.ymin_ * LONGLAT_RATIO);
    int ymax = (int) (extent.ymax_ * LONGLAT_RATIO);

    MeshGridObjectExt gridLB, gridRT;
    gridLB.setGridID(xmin, ymin, 4);
    gridRT.setGridID(xmax, ymax, 4);

    MeshDimension dim;
    gridLB.different(gridRT, dim);

    for (int i = 0; i < dim.width; i++) {
        for (int j = 0; j < dim.height; j++) {

            MeshGridObjectExt gridObj;
            gridObj.setGridID(xmin, ymin, 4);
            gridObj.offsetGrid(i, j);

            string gridName = gridObj.getGridName();

            int32_t mesh_id = MeshName2MeshID(gridName);
            mesh_id_map_.insert(make_pair(mesh_id, gridName));
            mesh_set_.insert(gridName);
        }
    }

    for (auto mesh_it : mesh_id_map_) {
        string meshname = mesh_it.second;

        unordered_map<string, shared_ptr<MeshObj>>::iterator it = meshs_.find(meshname);
        if (it == meshs_.end()) {
            string meshPath = GetMeshFullPath(meshname);
            shared_ptr<MeshObj> kdMesh = make_shared<MeshObj>();

            LOG(INFO) << "Prepare grid " << meshname;

            if (kdMesh->LoadMesh(meshname, meshPath, strtree_)) {
                meshs_.insert(make_pair(meshname, kdMesh));
            } else {
//                LOG(ERROR) << "Load mesh error. path '" << meshPath << "'";
                continue;
            }
        }
    }
    int roadcnt = 0;
    for(auto mesh_it : meshs_) {
        roadcnt += mesh_it.second->roads_.size();
    }

    int meshCount = dim.width * dim.height;
    LOG(INFO) << "Prepare mesh count " << meshCount;
    OutputHighWay(this);
    OutputHighNode(this);
    return true;

}

bool MeshManager::MeshOutOfRange(const string& mesh_name) {
     return mesh_set_.find(mesh_name) == mesh_set_.end();
}

shared_ptr<MeshObj> MeshManager::GetMesh(const KDExtent &extent) {
    int centx = (int) (extent.CenterX() * LONGLAT_RATIO);
    int centy = (int) (extent.CenterY() * LONGLAT_RATIO);

    MeshGridObjectExt grid;
    grid.setGridID(centx, centy, 4);
    string gridName = grid.getGridName();

    unordered_map<string, shared_ptr<MeshObj>>::iterator it = meshs_.find(gridName);
    if (it != meshs_.end()) {
        return it->second;
    }
    return nullptr;
}

shared_ptr<MeshObj> MeshManager::GetMesh(const string& mesh_id) {
    auto meshit = meshs_.find(mesh_id);
    if (meshit != meshs_.end()) {
        return meshit->second;
    }

    return nullptr;
}

shared_ptr<KDRoad> MeshManager::GetRoad(string mesh_id, int32_t road_id) {
    auto meshit = meshs_.find(mesh_id);
    if (meshit != meshs_.end()) {
        shared_ptr<MeshObj> mesh = meshit->second;

        auto roadit = mesh->roads_.find(road_id);
        if (roadit != mesh->roads_.end()) {
            return roadit->second;
        }
    }
    return nullptr;
}

string MeshManager::GetMeshFullPath(const string &meshId) {
    string mesh_data_path = GlobalCache::GetInstance()->mesh_data_path();
    string roadFile = mesh_data_path + "/" + meshId + "/";
    return roadFile;
}

bool MeshManager::GetAdjMeshNameList (const unordered_map<int32_t, string>& mesh_id_map,
                                   const string& mesh_name,
                                   vector<string>& mesh_list) {
    int32_t meshid = MeshName2MeshID(mesh_name);
    int32_t mapx = (meshid / 10000) * 8 + ((meshid / 10) % 10);
    int32_t mapy = ((meshid / 100) % 100) * 8 + (meshid % 10);
    int32_t leftmapx = mapx - 1;
    int32_t rightmapx = mapx + 1;
    int32_t topmapy = mapy + 1;
    int32_t bottommapy = mapy - 1;
    int32_t left = (leftmapx / 8) * 10000 + (mapy / 8) * 100 + (leftmapx % 8) * 10 + (mapy % 8);
    int32_t right = (rightmapx / 8) * 10000 + (mapy / 8) * 100 + (rightmapx % 8) * 10 + (mapy % 8);
    int32_t top = (mapx / 8) * 10000 + (topmapy / 8) * 100 + (mapx % 8) * 10 + (topmapy % 8);
    int32_t bottom = (mapx / 8) * 10000 + (bottommapy / 8) * 100 + (mapx % 8) * 10 + (bottommapy % 8);

    auto left_mesh = mesh_id_map.find(left);
    if (left_mesh != mesh_id_map.end())
        mesh_list.emplace_back(left_mesh->second);

    auto right_mesh = mesh_id_map.find(right);
    if (right_mesh != mesh_id_map.end())
        mesh_list.emplace_back(right_mesh->second);

    auto top_mesh = mesh_id_map.find(top);
    if (top_mesh != mesh_id_map.end())
        mesh_list.emplace_back(top_mesh->second);

    auto bottom_mesh = mesh_id_map.find(bottom);
    if (bottom_mesh != mesh_id_map.end())
        mesh_list.emplace_back(bottom_mesh->second);

    return !mesh_list.empty();
}

int32_t MeshManager::MeshName2MeshID(const string& meshid) {
    if(meshid.length() != 10){
        printf("meshid format error!");
        return 0;
    }
    int res = 0;
    double lat = 0.0;
    double lon = 0.0;
    lat = (meshid.c_str()[0] - 'A') * 6.0;
    lon = (atoi(meshid.substr(1, 2).c_str()) - 31) * 6.0 - 60;

    lat += (49 - atoi(meshid.substr(4, 3).c_str()) - 1) / 8.0;
    lon += (atoi(meshid.substr(7, 3).c_str()) - 1) / 8.0;

    res = (int)lat;
    res = res * 100 + (int)lon;
    res = res * 10 + (int)((lat - (int)lat) * 8.0);
    res = res * 10 + (int)((lon - (int)lon) * 8.0);
    return res;
}