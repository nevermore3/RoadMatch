//
// Created by liujian on 19-7-9.
//

#include "diff_data_manager.h"
#include "global_cache.h"
#include "data_manager/mesh_manager.h"
#include "geos/geom/Coordinate.h"
#include "geos/geom/GeometryFactory.h"
#include "geos/geom/CoordinateSequence.h"
#include "geos/geom/CoordinateArraySequence.h"
#include "geos/geom/Point.h"

#include <mvg/Coordinates.hpp>

#include "util/match_geo_util.h"
#include "util/geometry_util.h"
#include "util/maccoord_offset.h"
#include "util/morton_code.h"
#include "util/distance.h"

#include "glog/logging.h"
#include "glog/log_severity.h"
#include "data_types.h"

#include <shp/ShpData.hpp>
#include <string>
#include <unordered_map>

#include <sys/types.h>
#include <sys/stat.h>
using namespace std;
using namespace kd::automap;

bool DiffDataManager::LoadData(const KDExtent &extent)
{
    string diff_data_path = GlobalCache::GetInstance()->diff_data_path();
    string roadFile = diff_data_path + "/" + "route_beijingshi_link";

    //load road
    if (!LoadRoad(roadFile, strtree_)) {
        LOG(ERROR) << "Load road error.";
        return false;
    }

    //load node
    string nodeFile = diff_data_path + "/" + "route_beijingshi_node";
    if (!LoadNode(nodeFile)) {
        LOG(ERROR) << "Load node error.";
        return false;
    }
    BuildBoundry();
    //RemoveNonHighWay();
    string output = GlobalCache::GetInstance()->out_path();
    OutputRoad(output);
    OutputNode(output);

    return true;
}



bool DiffDataManager::LoadRoad(string fileName, shared_ptr<STRtree> strtree)
{
    ShpData shpData(fileName);
    if (!shpData.isInit()) {
        return false;
    }

    const geos::geom::GeometryFactory *gf = geos::geom::GeometryFactory::getDefaultInstance();

    int recordNums = shpData.getRecords();
    LOG(INFO)<<"record nums is :"<<recordNums;

    for (size_t i = 0; i < recordNums; i++) {
        SHPObject *shpObject = shpData.readShpObject(i);
        if (!shpObject || shpObject->nSHPType != SHPT_ARC) {
            continue;
        }
        //read road
        shared_ptr<KDRoad> road = make_shared<KDRoad>();
        //read att
        road->id_ = shpData.readIntField(i, "id");
        road->road_name_ = shpData.readStringField(i, "name");
        road->mesh_id_ = shpData.readStringField(i, "block_id");
        road->f_node_id_ = road->t_node_id_ = -1;
        road->direction_ = 1;

        //remove non-highway
//        if (road->road_name_.find("高速") == std::string::npos) {
//            continue;
//        }

        shared_ptr<MeshObj>meshObj;
        if (meshs_.find(road->mesh_id_) == meshs_.end()) {
            meshObj= make_shared<MeshObj>();
        } else {
            meshObj = meshs_[road->mesh_id_];
        }

        double roadLength = 0;
        //read road geometry
        int vertexNums = shpObject->nVertices;
        road->points_.resize(vertexNums);
        for (size_t j = 0; j < vertexNums; j++) {
            road->points_[j] = make_shared<KDCoord>();
            road->points_[j]->lng_ = shpObject->padfX[j];
            road->points_[j]->lat_ = shpObject->padfY[j];
            road->points_[j]->z_ = shpObject->padfZ[j];

            //code(node) <---> road
            int64_t mortonCode = MortonCode::GetMortonCodeFromRAWCoord(road->points_[j]->lng_, road->points_[j]->lat_);

            if (code_road_.find(mortonCode) != code_road_.end()) {
                code_road_[mortonCode].insert(road);
            } else {
                set<shared_ptr<KDRoad>> link;
                link.insert(road);
                code_road_.insert(make_pair(mortonCode, link));
            }
            //add from_node
            if (j == 0) {
                road->f_node_id_ = mortonCode;
            }
            // add to_node
            if (j == vertexNums - 1) {
                road->t_node_id_ = mortonCode;
            }
            if (j != 0) {
                roadLength += Distance::distance(road->points_[j-1], road->points_[j]);
            }
        }
        road->length_ = roadLength / 100;
        //add to cache
        CoordinateSequence *cl = new CoordinateArraySequence();
        for (int j = 0; j < vertexNums; j++) {

            double utmX, utmY;
            char zone[4] = {0};
            Coordinates::ll2utm(road->points_[j]->lat_, road->points_[j]->lng_, utmX, utmY, zone);

            cl->add(Coordinate(utmX, utmY, shpObject->padfZ[j]));
        }

        road->line_.reset(gf->createLineString(cl));

        //      quadtree->insert(road->line_->getEnvelopeInternal(), road.get());
        strtree->insert(road->line_->getEnvelopeInternal(), road.get());
        //add to cache


        meshObj->roads_.insert(make_pair(road->id_, road));
        if (meshs_.find(road->mesh_id_) == meshs_.end()) {
            meshs_.insert(make_pair(road->mesh_id_, meshObj));
        }
    }
    return true;
}


vector<int32_t> split(const string &s)
{
    const char *delims = ",";
    vector<int32_t>tokens;
    int len = s.size();
    char *tok;
    char *line = (char*)malloc(len + 1);
    memset(line, 0, len + 1);
    strcpy(line, s.c_str());

    tok = strtok(line, delims);
    while (tok != nullptr)
    {
        tokens.push_back(atoi(tok));
        tok = strtok(nullptr, delims);
    }
    return tokens;
}

string ReverseString(const string &str)
{
    string result;
    std::size_t  found = str.find('_');
    if (found == std::string::npos) {
        return result;
    }
    string pre = str.substr(0, found);
    string post = str.substr(found+1);
    result = post  + "_" + pre;
    return result;
}


void DiffDataManager::StringToVector(string str, shared_ptr<KDRoadNode> node, shared_ptr<MeshObj> meshObj)
{
    vector<shared_ptr<KDRoad>> fromRoad;
    int64_t mortonCode = MortonCode::GetMortonCodeFromRAWCoord(node->coord_.lng_, node->coord_.lat_);
    if (code_road_.find(mortonCode) != code_road_.end()) {
        set<shared_ptr<KDRoad>>codeRoad = code_road_[mortonCode];
        for (auto i : codeRoad) {
            if (i->mesh_id_ == node->mesh_id_) {
                fromRoad.push_back(i);
            }
        }
    }
    node->from_roads_ = node->to_roads_= fromRoad;
}


bool DiffDataManager::LoadNode(string file_name)
{
    ShpData shpData(file_name);
    if (!shpData.isInit()) {
        return false;
    }

    int recordNums = shpData.getRecords();
    for (size_t i = 0; i < recordNums; i++) {

        SHPObject *shpObject = shpData.readShpObject(i);
        if (!shpObject || shpObject->nSHPType != SHPT_POINT)
            continue;

        shared_ptr<KDRoadNode> node = make_shared<KDRoadNode>();

        //read att
        node->id_ = shpData.readIntField(i, "id");
        node->mesh_id_ = ReverseString(shpData.readStringField(i, "block_id"));
        node->adj_id_ = -1;
        node->boundary_ = 0;

        node->coord_.lng_ = shpObject->padfX[0];
        node->coord_.lat_ = shpObject->padfY[0];
        node->coord_.z_ = shpObject->padfZ[0];

        shared_ptr<MeshObj> meshObj;
        if (meshs_.find(node->mesh_id_) == meshs_.end()) {
            meshObj= make_shared<MeshObj>();
        } else {
            meshObj = meshs_[node->mesh_id_];
        }

        string str = shpData.readStringField(i, "link_ids");
        StringToVector(str, node, meshObj);

        if (node->from_roads_.empty()) {
            //remove non-way point
            continue;
        }
        //read geometry
        if (shpObject->nVertices != 1) {
            continue;
        }

        int64_t mortonCode = MortonCode::GetMortonCodeFromRAWCoord(node->coord_.lng_, node->coord_.lat_);

        size_t  size = node->from_roads_.size();

        if (code_node_.find(mortonCode) == code_node_.end()) {
            set<shared_ptr<KDRoadNode>>codeNode;
            codeNode.insert(node);
            code_node_.insert(make_pair(mortonCode, codeNode));
        } else {
            code_node_[mortonCode].insert(node);
        }

        //add to cache
        meshObj->road_nodes_.insert(make_pair(node->id_, node));
        if (meshs_.find(node->mesh_id_) == meshs_.end()) {
            meshs_.insert(make_pair(node->mesh_id_, meshObj));
        }
    }
    return true;
}

void DiffDataManager::BuildBoundry()
{
    //cout<<code_node_.size()<<endl;
    int count = 0;
    for(auto i : code_node_) {

        set<shared_ptr<KDRoadNode>>codeNode = i.second;
        int64_t code = i.first;

        if (codeNode.size() != 2) {
            continue;
        }
        count++;
        shared_ptr<KDRoadNode> nodeA = *(codeNode.begin());
        shared_ptr<KDRoadNode> nodeB = *(codeNode.rbegin());

        nodeA->boundary_ = 1;
        nodeB->boundary_ = 1;
        nodeA->adj_id_ = nodeB->id_;
        nodeB->adj_id_ = nodeA->id_;
        nodeA->adj_mesh_id_ = nodeB->mesh_id_;
        nodeB->adj_mesh_id_ = nodeA->mesh_id_;
    }
    //cout<<"count  is "<<count<<endl;
    // 修正 road的 f_node_id & t_node_id
    DiffDataManager *diffManage = DiffDataManager::GetInstance();
    unordered_map<string, shared_ptr<MeshObj>> meshs = diffManage->meshs_;
    for (auto i : meshs) {
        string meshID = i.first;
        shared_ptr<MeshObj>meshObj = i.second;
        unordered_map<int32_t, shared_ptr<KDRoad>> roads = meshObj->roads_;
        for (auto road : roads) {
            shared_ptr<KDRoad>roadObj = road.second;
            int64_t  fcode = roadObj->f_node_id_;

            set<shared_ptr<KDRoadNode>> fnodes = code_node_[fcode];
            for (auto j : fnodes) {
                if (j->boundary_ == 1) {
                    if (meshID == j->mesh_id_) {
                        roadObj->f_node_id_ = j->id_;
                    }
                } else {
                    roadObj->f_node_id_ = j->id_;
                }
            }

            int64_t  tcode = roadObj->t_node_id_;
            set<shared_ptr<KDRoadNode>> tnodes = code_node_[tcode];
            for (auto j : tnodes) {
                if (j->boundary_ == 1) {
                    if (meshID == j->mesh_id_) {
                        roadObj->t_node_id_ = j->id_;
                    }
                } else {
                    roadObj->t_node_id_ = j->id_;
                }
            }

        }

    }

}


string VectorToString(vector<shared_ptr<KDRoad>> linkRoad)
{
    string result;
    for (auto i : linkRoad) {
        result += to_string(i->id_);
        result += ",";
    }
    return result;
}

void DiffDataManager::OutputNode(const string &filename)
{
    if (access(filename.c_str(), NULL) == -1) {
        if (mkdir(filename.c_str(), 0755) == -1) {
            LOG(ERROR)<<"mkdir output error !!!";
            return;
        }
    }
    string dbfName = filename + "/NODE.dbf";
    string shpName = filename + "/NODE.shp";

    SHPHandle ptrRoadShp_ = nullptr;
    DBFHandle ptrRoadDbf_ = nullptr;

    ptrRoadShp_ = SHPCreate(shpName.c_str(), SHPT_POINTZ);
    ptrRoadDbf_ = DBFCreate(dbfName.c_str());

    DBFAddField(ptrRoadDbf_, "ID", FTLong, 16, 0);
    DBFAddField(ptrRoadDbf_, "meshID", FTString, 64, 0);
    DBFAddField(ptrRoadDbf_, "from_road", FTString, 64, 0);
    DBFAddField(ptrRoadDbf_, "to_road", FTString, 64, 0);
    DBFAddField(ptrRoadDbf_, "boundry", FTLong, 16, 0);
    DBFAddField(ptrRoadDbf_, "adj_mesh", FTString, 64, 0);
    DBFAddField(ptrRoadDbf_, "adj_id", FTLong, 16, 0);


    DiffDataManager *manager = DiffDataManager::GetInstance();

    int nCount = 0;
    for (auto mesh : meshs_) {
        shared_ptr<MeshObj> meshobj = mesh.second;

        for (auto i: meshobj->road_nodes_) {
            shared_ptr<KDRoadNode> nodeobj = i.second;

            int coord_nums = 1;
            double *coords_x = new double[coord_nums];
            double *coords_y = new double[coord_nums];
            double *coords_z = new double[coord_nums];

            for (int j = 0; j < coord_nums; j++) {
                coords_x[j] = nodeobj->coord_.lng_;
                coords_y[j] = nodeobj->coord_.lat_;
                coords_z[j] = nodeobj->coord_.z_;
            }

            SHPObject *shpObj = SHPCreateSimpleObject(SHPT_POINTZ, 1, coords_x, coords_y, coords_z);
            nCount = SHPWriteObject(ptrRoadShp_, -1, shpObj);
            delete[] coords_x;
            delete[] coords_y;
            delete[] coords_z;

            DBFWriteLongAttribute(ptrRoadDbf_, nCount, 0, nodeobj->id_);
            DBFWriteStringAttribute(ptrRoadDbf_, nCount, 1, nodeobj->mesh_id_.c_str());
            DBFWriteStringAttribute(ptrRoadDbf_, nCount, 2, VectorToString(nodeobj->from_roads_).c_str());
            DBFWriteStringAttribute(ptrRoadDbf_, nCount, 3, VectorToString(nodeobj->to_roads_).c_str());
            DBFWriteLongAttribute(ptrRoadDbf_, nCount, 4, nodeobj->boundary_);
            DBFWriteStringAttribute(ptrRoadDbf_, nCount, 5, nodeobj->adj_mesh_id_.c_str());
            DBFWriteLongAttribute(ptrRoadDbf_, nCount, 6, nodeobj->adj_id_);

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


void DiffDataManager::OutputRoad(const string &filename){
    if (access(filename.c_str(), NULL) == -1) {
        if (mkdir(filename.c_str(), 0755) == -1) {
            LOG(ERROR)<<"mkdir output error !!!";
            return;
        }
    }
    string dbfName = filename + "/ROAD.dbf";
    string shpName = filename + "/ROAD.shp";

    SHPHandle ptrRoadShp_ = nullptr;
    DBFHandle ptrRoadDbf_ = nullptr;

    ptrRoadShp_ = SHPCreate(shpName.c_str(), SHPT_ARCZ);
    ptrRoadDbf_ = DBFCreate(dbfName.c_str());

    DBFAddField(ptrRoadDbf_, "ID", FTLong, 16, 0);
    DBFAddField(ptrRoadDbf_, "from_node", FTLong, 16, 0);
    DBFAddField(ptrRoadDbf_, "to_node", FTLong, 16, 0);
    DBFAddField(ptrRoadDbf_, "name", FTString, 64, 0);
    DBFAddField(ptrRoadDbf_, "length", FTDouble, 8, 2);
    DBFAddField(ptrRoadDbf_, "meshId", FTString, 64, 0);

    DiffDataManager *manager = DiffDataManager::GetInstance();
    int nCount = 0;
    for (auto mesh : meshs_) {
        shared_ptr<MeshObj> meshobj = mesh.second;
        for (auto i: meshobj->roads_) {

            shared_ptr<KDRoad>road = i.second;
            size_t coord_nums = road->points_.size();
            if (coord_nums == 0) {
                LOG(ERROR)<<" coord nums is  0";
                continue;
            }
            double *coords_x = new double[coord_nums];
            double *coords_y = new double[coord_nums];
            double *coords_z = new double[coord_nums];

            for (int j = 0; j < coord_nums; j++) {
                coords_x[j] = road->points_[j]->lng_;
                coords_y[j] = road->points_[j]->lat_;
                coords_z[j] = road->points_[j]->z_;
            }

            SHPObject *shpObj = SHPCreateSimpleObject(SHPT_ARCZ, (int) coord_nums, coords_x, coords_y, coords_z);
            nCount = SHPWriteObject(ptrRoadShp_, -1, shpObj);
            delete[] coords_x;
            delete[] coords_y;
            delete[] coords_z;

            DBFWriteLongAttribute(ptrRoadDbf_, nCount, 0, road->id_);
            DBFWriteLongAttribute(ptrRoadDbf_, nCount, 1, road->f_node_id_);
            DBFWriteLongAttribute(ptrRoadDbf_, nCount, 2, road->t_node_id_);
            DBFWriteDoubleAttribute(ptrRoadDbf_, nCount, 4, road->length_);
            DBFWriteStringAttribute(ptrRoadDbf_, nCount, 5, road->mesh_id_.c_str());
            DBFWriteStringAttribute(ptrRoadDbf_, nCount, 3, road->road_name_.c_str());

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

shared_ptr<MeshObj> DiffDataManager::GetMesh(const string& mesh_id) {
    auto meshit = meshs_.find(mesh_id);
    if (meshit != meshs_.end()) {
        return meshit->second;
    }

    return nullptr;
}

shared_ptr<KDRoad> DiffDataManager::GetRoad(string mesh_id, int32_t road_id) {
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