#include <iostream>
#include <unistd.h>

//#include "coord_offset/coordoffset.h"

#include "CMesh.h"
//#include "util/CommonUtil.h"

//#include "geom/geo_util.h"
#include "geos/geom/GeometryFactory.h"
#include "geos/geom/CoordinateArraySequence.h"
#include "geos/geom/Point.h"
#include "geos/geom/LineString.h"

#include "Poco/JSON/Parser.h"
#include "Poco/Path.h"
#include "Poco/File.h"

using namespace Poco;
using namespace Poco::JSON;

#define ADD_NEW_ROAD 0

namespace kd{
    namespace autohdmap{
        CMesh *gpMesh = nullptr;

        CMesh::CMesh() {
            g_kvMesh.clear();
            g_kvCalcMesh.clear();
            g_meshRoads.clear();
        }

        CMesh::~CMesh() {
            for (auto iter:g_kvMesh){
                shared_ptr<MeshRectangle> pRect = iter.second;
                pRect.reset();
            }
            g_kvMesh.clear();

            for (auto iter:g_kvCalcMesh){
                shared_ptr<MeshRectangle> pRect = iter.second;
                pRect.reset();
            }
            g_kvCalcMesh.clear();

            for (auto iter:g_meshRoads){
                vector<shared_ptr<stRoad>> arrRoads = iter.second;
                for(auto& pRoad:arrRoads){
                    pRoad.reset();
                }
            }
            g_meshRoads.clear();
            g_meshRoadIds.clear();
            g_meshBridgeRoads.clear();
            g_meshMainRoads.clear();

            g_meshBridgeBoundarys.clear();
        }


        std::string CMesh::getMeshId(const double& longitude, const double& latitude) const {
            if (longitude < -180 || longitude > 180 or latitude < -90 or latitude > 90){
                return std::string();
            }
            MeshGridObjectExt gridObjectExt;
            gridObjectExt.setGridID((int) (longitude * 3600000), (int) (latitude * 3600000), 4);
            string strMeshId = gridObjectExt.getGridName();

            return strMeshId;
        }

        void CMesh::getMeshList(const double *coordinates, const int& ptCount, std::vector<std::string> &meshList) const {
            if (coordinates == nullptr){
                return;
            }

            // check coordinate
            for (int i=0; i<ptCount; i++){
                double dx = coordinates[i * 2 + 0];
                double dy = coordinates[i * 2 + 1];

                if (dx < -180 || dx > 180 or dy < -90 or dy > 90){
                    return;
                }
            }

            // get meshId
            for (int i=0; i<ptCount; i++){
                double dx = coordinates[i * 2];
                double dy = coordinates[i * 2 + 1];

                std::string strMeshId = this->getMeshId(dx, dy);
                meshList.emplace_back(strMeshId);
            }
        }

        CMesh* CMesh::getInstance() {
            if(gpMesh) {
                return gpMesh;
            } else {
                gpMesh = new CMesh();
                return gpMesh;
            }
        }

        void CMesh::setCityRegion(const string& dataDir, const string &cityCode,
                const double &dxmin, const double &dymin,
                const double &dxmax, const double &dymax) {
            dataDirectory_ = dataDir;
            cityCode_ = cityCode;
            dxmin_ = dxmin;
            dymin_ = dymin;
            dxmax_ = dxmax;
            dymax_ = dymax;
        }

        void CMesh::setCityInfo(const string& dataDir, const string &cityCode) {
            dataDirectory_ = dataDir;
            cityCode_ = cityCode;
        }

        bool CMesh::initMeshByList(const int& meshLevel, const vector<string>& meshList) {
//mark for compile
//            MeshGridObjectExt gridObjectExt;
//
//            for (const auto& mesh_name: meshList){
//                gridObjectExt.setGridName(mesh_name);
//
//                std::shared_ptr<MeshRectangle> pRect = make_shared<MeshRectangle>();
//                gridObjectExt.getGridArea(*pRect);
//
//                double xmin = (double)pRect->xmin / 3600000.00;
//                double ymin = (double)pRect->ymin / 3600000.00;
//                double xmax = (double)pRect->xmax / 3600000.00;
//                double ymax = (double)pRect->ymax / 3600000.00;
//
//                g_kvMesh.insert(std::make_pair(mesh_name, pRect));
//
//                std::shared_ptr<MeshRectangle> pNewRect = make_shared<MeshRectangle>();
//                double xstep = CommonUtil::UT_GetMapDistance(xmin, ymin, xmin+0.01, ymin);
//                double ystep = CommonUtil::UT_GetMapDistance(xmin, ymin, xmin, ymin+0.01);
//
//                // 外扩10m
//                double new_xmin = xmin - 10.00 / xstep * 0.01;
//                double new_ymin = ymin - 10.00 / ystep * 0.01;
//
//                double new_xmax = xmax + 10.00 / xstep * 0.01;
//                double new_ymax = ymax + 10.00 / ystep * 0.01;
//
//                pNewRect->xmin = (int)(new_xmin * 3600000);
//                pNewRect->ymin = (int)(new_ymin * 3600000);
//                pNewRect->xmax = (int)(new_xmax * 3600000);
//                pNewRect->ymax = (int)(new_ymax * 3600000);
//
//                g_kvCalcMesh.insert(std::make_pair(mesh_name, pNewRect));
//            }
//            Poco::Logger::get("FullAutoLogger").information("total mesh count:" + std::to_string(g_kvMesh.size()));
//
//            std::string provinceCode = cityCode_.substr(0, 2);
//            provinceCode.append("0000");
//            std::string filePath = dataDirectory_ + "/" + provinceCode;
//
//            vector<string> sub_dir_names;
//            kd::autohdmap::CommonUtil::showAllFiles(filePath.data(), sub_dir_names);
//            for (const auto& test_mesh_name: sub_dir_names){
//                bool findMesh = false;
//                for(const auto& meshId: meshList){
//                    if (meshId == test_mesh_name){
//                        findMesh = true;
//                        break;
//                    }
//                }
//                if (!meshList.empty() && !findMesh){
//                    continue;
//                }
//                initMeshRoad(filePath.data(), test_mesh_name.data());
//            }

            return true;
        }

        bool CMesh::saveMeshAsShape(const vector<string> &meshList, const string& fileName) {

//mark for compile
//            MeshGridObjectExt gridObjectExt;
//            set<string> done_meshes;
//
//            string shapePath = "./" + fileName + ".shp";
//            if (access(shapePath.data(), F_OK) != -1){
//                Poco::Logger::get("FullAutoLogger").error(shapePath + " is not exist");
//                return true;
//            }
//            MTL::SHPHandle *pShp = MTL::SHPCreate(shapePath.data(), 3);
//            string dbfPath = "./" + fileName + ".dbf";
//            MTL::DBFHandle *pDbf = MTL::DBFCreate(dbfPath.data());
//
//            MTL::DBFAddField(pDbf, "Name", 0, 32, 0);
//            int colIndex = MTL::DBFGetFieldIndex(pDbf, "Name");
//
//            //开始写入
//            int _record = 0;
//            for (const auto& mesh_name: meshList){
//                if (mesh_name.length() != 10){
//                    continue;
//                }
//                if (done_meshes.find(mesh_name) != done_meshes.end()){
//                    continue;
//                }
//                gridObjectExt.setGridName(mesh_name);
//
//                std::shared_ptr<MeshRectangle> pRect = make_shared<MeshRectangle>();
//                gridObjectExt.getGridArea(*pRect);
//
//                double xmin = (double)pRect->xmin / 3600000.00;
//                double ymin = (double)pRect->ymin / 3600000.00;
//                double xmax = (double)pRect->xmax / 3600000.00;
//                double ymax = (double)pRect->ymax / 3600000.00;
//
//                MTL::SHPObject *pMeshObj = MTL::SHPInitialObject();
//                auto *pCoord = new double[10];
//                pCoord[0] = xmin;
//                pCoord[1] = ymin;
//                pCoord[2] = xmax;
//                pCoord[3] = ymin;
//                pCoord[4] = xmax;
//                pCoord[5] = ymax;
//                pCoord[6] = xmin;
//                pCoord[7] = ymax;
//                pCoord[8] = xmin;
//                pCoord[9] = ymin;
//                MTL::SHPCreateSimpleObject(pMeshObj, 3, 5, pCoord);
//                MTL::SHPWriteObject(pShp, pMeshObj, _record);
//                MTL::DBFWriteStringAttribute(pDbf, _record, 0, (void*)mesh_name.data());
//
//                delete []pCoord;
//                MTL::SHPReleaseObject(pMeshObj);
//
//                done_meshes.insert(mesh_name);
//                _record++;
//            }
//
//            MTL::SHPClose(pShp);
//            MTL::DBFClose(pDbf);

            return true;
        }

        bool CMesh::saveBridgeAsShape(const string& fileName) {
//mark for compile
//            string shapePath = "./" + fileName + ".shp";
//            if (access(shapePath.data(), F_OK) != -1){
//                return true;
//            }
//            MTL::SHPHandle *pShp = MTL::SHPCreate(shapePath.data(), 3);
//            string dbfPath = "./" + fileName + ".dbf";
//            MTL::DBFHandle *pDbf = MTL::DBFCreate(dbfPath.data());
//
//            MTL::DBFAddField(pDbf, "Name", 0, 32, 0);
//            int colIndex = MTL::DBFGetFieldIndex(pDbf, "Name");
//
//            //开始写入
//            int _record = 0;
//            for (const auto& mesh_it: g_meshBridgeBoundarys){
//                string mesh_name = mesh_it.first;
//
//                int bridgeIndex = 0;
//                int bridgeCount = 0;
//                this->getBridgeCount(mesh_name, bridgeCount);
//                for (int i=0; i<bridgeCount; i++){
//                    double boundary[4] = {0};
//                    this->getBridgeBoundary(mesh_name, i, boundary);
//                    bridgeIndex++;
//                    string bridgeName = mesh_name + "_" + std::to_string(bridgeIndex);
//                    double xmin = boundary[0];
//                    double ymin = boundary[1];
//                    double xmax = boundary[2];
//                    double ymax = boundary[3];
//
//                    MTL::SHPObject *pMeshObj = MTL::SHPInitialObject();
//                    auto pCoord = new double[10];
//                    pCoord[0] = xmin;
//                    pCoord[1] = ymin;
//                    pCoord[2] = xmax;
//                    pCoord[3] = ymin;
//                    pCoord[4] = xmax;
//                    pCoord[5] = ymax;
//                    pCoord[6] = xmin;
//                    pCoord[7] = ymax;
//                    pCoord[8] = xmin;
//                    pCoord[9] = ymin;
//                    MTL::SHPCreateSimpleObject(pMeshObj, 3, 5, pCoord);
//                    MTL::SHPWriteObject(pShp, pMeshObj, _record);
//                    MTL::DBFWriteStringAttribute(pDbf, _record, 0, (void*)bridgeName.data());
//
//                    delete []pCoord;
//                    MTL::SHPReleaseObject(pMeshObj);
//
//                    _record++;
//                }
//            }
//
//            MTL::SHPClose(pShp);
//            MTL::DBFClose(pDbf);

            return true;
        }

        bool CMesh::saveMeshAsGeoJson(const vector<string> &meshList, const string& fileName) {
            MeshGridObjectExt gridObjectExt;
            set<string> done_meshes;

            string filePath = "./" + fileName + ".geojson";
            if (access(filePath.data(), F_OK) != -1){
                return true;
            }

            Poco::JSON::Object obj;
            obj.set("type", "FeatureCollection");
            Poco::JSON::Array feature_arr;

            //开始写入
            for (const auto& mesh_name: meshList){
                if (mesh_name.length() != 10){
                    continue;
                }
                if (done_meshes.find(mesh_name) != done_meshes.end()){
                    continue;
                }
                gridObjectExt.setGridName(mesh_name);

                std::shared_ptr<MeshRectangle> pRect = make_shared<MeshRectangle>();
                gridObjectExt.getGridArea(*pRect);

                double xmin = (double)pRect->xmin / 3600000.00;
                double ymin = (double)pRect->ymin / 3600000.00;
                double xmax = (double)pRect->xmax / 3600000.00;
                double ymax = (double)pRect->ymax / 3600000.00;

                Poco::JSON::Array arrCoords;

                double dx = xmin;
                double dy = ymin;
                for (int j=0; j<5; j++){
                    if (j==0) {
                        dx = xmin;
                        dy = ymin;
                    }
                    else if(j == 1) {
                        dx = xmin;
                        dy = ymax;
                    }
                    else if (j == 2) {
                        dx = xmax;
                        dy = ymax;
                    }
                    else if (j == 3) {
                        dx = xmax;
                        dy = ymin;
                    }
                    else if (j == 4) {
                        dx = xmin;
                        dy = ymin;
                    }
                    Poco::JSON::Array arrCoord;
                    arrCoord.add(dx);
                    arrCoord.add(dy);
                    arrCoords.add(arrCoord);
                }

                Poco::JSON::Array polygon;
                polygon.add(arrCoords);

                Poco::JSON::Object prop_obj;
                prop_obj.set("Name", mesh_name);

                Poco::JSON::Object geometry_obj;
                geometry_obj.set("type", "Polygon");
                geometry_obj.set("coordinates", polygon);

                Poco::JSON::Object mesh_obj;
                mesh_obj.set("properties", prop_obj);
                mesh_obj.set("type", "Feature");
                mesh_obj.set("geometry", geometry_obj);

                feature_arr.add(mesh_obj);

                done_meshes.insert(mesh_name);
            }
            obj.set("features", feature_arr);

            stringstream jsnString;
            obj.stringify(jsnString);
            std::string json_data = jsnString.str();

            std::fstream fs;
            fs.open(filePath, std::fstream::out | std::fstream::trunc);
            //
            if (fs.is_open()) {
                fs << json_data;
                //
                fs.close();
            }

            return true;
        }

        bool CMesh::saveBridgeAsGeoJson(const string &fileName) {
            string filePath = "./" + fileName + ".geojson";
            if (access(filePath.data(), F_OK) != -1){
                return true;
            }

            Poco::JSON::Object obj;
            obj.set("type", "FeatureCollection");
            Poco::JSON::Array feature_arr;

            //开始写入
            for (const auto& mesh_it: g_meshBridgeBoundarys){
                string mesh_name = mesh_it.first;

                int bridgeIndex = 0;
                int bridgeCount = 0;
                this->getBridgeCount(mesh_name, bridgeCount);
                for (int i=0; i<bridgeCount; i++){
                    double boundary[4] = {0};
                    this->getBridgeBoundary(mesh_name, i, boundary);
                    bridgeIndex++;
                    string bridgeName = mesh_name + "_" + std::to_string(bridgeIndex);
                    double xmin = boundary[0];
                    double ymin = boundary[1];
                    double xmax = boundary[2];
                    double ymax = boundary[3];

                    Poco::JSON::Array arrCoords;

                    double dx = xmin;
                    double dy = ymin;
                    for (int j=0; j<5; j++){
                        if (j==0) {
                            dx = xmin;
                            dy = ymin;
                        }
                        else if(j == 1) {
                            dx = xmin;
                            dy = ymax;
                        }
                        else if (j == 2) {
                            dx = xmax;
                            dy = ymax;
                        }
                        else if (j == 3) {
                            dx = xmax;
                            dy = ymin;
                        }
                        else if (j == 4) {
                            dx = xmin;
                            dy = ymin;
                        }
                        Poco::JSON::Array arrCoord;
                        arrCoord.add(dx);
                        arrCoord.add(dy);
                        arrCoords.add(arrCoord);
                    }

                    Poco::JSON::Array polygon;
                    polygon.add(arrCoords);

                    Poco::JSON::Object prop_obj;
                    prop_obj.set("Name", bridgeName);

                    Poco::JSON::Object geometry_obj;
                    geometry_obj.set("type", "Polygon");
                    geometry_obj.set("coordinates", polygon);

                    Poco::JSON::Object mesh_obj;
                    mesh_obj.set("properties", prop_obj);
                    mesh_obj.set("type", "Feature");
                    mesh_obj.set("geometry", geometry_obj);

                    feature_arr.add(mesh_obj);
                }
            }
            obj.set("features", feature_arr);

            stringstream jsnString;
            obj.stringify(jsnString);
            std::string json_data = jsnString.str();

            fstream fs;
            fs.open(filePath, std::fstream::out | std::fstream::trunc);
            //
            if (fs.is_open()) {
                fs << json_data;
                //
                fs.close();
            }

            return true;
        }

        bool CMesh::initMeshRoad(const char *filePath, const char *mesh_name) {
//mark for compile
//            fromNodes.clear();
//            toNodes.clear();
//            fromBridgeNodes.clear();
//            toBridgeNodes.clear();
//
//            string strMeshId = mesh_name;
//            std::string mesh_path = string(filePath) + "/" + mesh_name;
//            std::string road_shp_path = mesh_path + "/" + "Road.shp";
//            std::string road_dbf_path = mesh_path + "/" + "Road.dbf";
//
//            // 确认文件是否存在
//            if (access(road_shp_path.data(), F_OK) || access(road_dbf_path.data(), F_OK)){
//                Poco::Logger::get("FullAutoLogger").error(road_shp_path + " is not exist");
//                return false;
//            }
//
//            shared_ptr<geos::index::quadtree::Quadtree> road_spatial_index = make_shared<geos::index::quadtree::Quadtree>();
//            const geos::geom::GeometryFactory *gf = geos::geom::GeometryFactory::getDefaultInstance();
//
//            map<int, vector<shared_ptr<stRoad>>> group_bridges;
//            int group_index = 0;
//            set<string> done_roads;
//
//            MTL::SHPHandle *pShpHandle = MTL::SHPOpen(road_shp_path.data(), "r");
//            MTL::DBFHandle *pDbfHandle = MTL::DBFOpen(road_dbf_path.data(), "r");
//
//            int road_class_index = MTL::DBFGetFieldIndex(pDbfHandle, "ROADCLASS");
//            int formway_index = MTL::DBFGetFieldIndex(pDbfHandle, "FORMWAY");
//            int road_id_index = MTL::DBFGetFieldIndex(pDbfHandle, "ROAD_ID");
//            int from_node_index = MTL::DBFGetFieldIndex(pDbfHandle, "FNODE_");
//            int to_node_index = MTL::DBFGetFieldIndex(pDbfHandle, "TNODE_");
//
//            const auto& mesh_iter = g_meshRoads.find(strMeshId);
//            if (mesh_iter == g_meshRoads.end()){
//                vector<shared_ptr<stRoad>> arrRoads;
//                g_meshRoads.insert(std::make_pair(strMeshId, arrRoads));
//            }
//
//            double buffer = 0.00025;//25m
//            int rec_count = pDbfHandle->nRecordNum;
//            for(int i=0; i<rec_count; i++){
//                MTL::SHPObject *pObject = MTL::SHPInitialObject();
//                int read_result = MTL::SHPReadObject(pShpHandle, pObject,i);
//                if (!read_result){
//                    MTL::SHPReleaseObject(pObject);
//                    continue;
//                }
//
//                std::string strRoadClass = MTL::DBFReadStringAttribute(pDbfHandle, i, road_class_index);
//                std::string strRoadId = MTL::DBFReadStringAttribute(pDbfHandle, i, road_id_index);
//                int formway = MTL::DBFReadIntegerAttribute(pDbfHandle, i, formway_index);
//                int from_node = MTL::DBFReadIntegerAttribute(pDbfHandle, i, from_node_index);
//                int to_node = MTL::DBFReadIntegerAttribute(pDbfHandle, i, to_node_index);
//
//                if (formway == 7){
//                    // 辅路不考虑
//                    MTL::SHPReleaseObject(pObject);
//                    continue;
//                }
//
//                if (formway == 3 || formway == 6 || formway == 8 || formway == 53 || formway == 56 || formway == 58){
//                    shared_ptr<stRoad> roadPtr = make_shared<stRoad>();
//                    roadPtr->roadClass = strRoadClass;
//                    roadPtr->roadId = strRoadId;
//                    roadPtr->formway = formway;
//                    roadPtr->fromNode = from_node;
//                    roadPtr->toNode = to_node;
//
//                    fromBridgeNodes[from_node].emplace_back(roadPtr->roadId);
//                    toBridgeNodes[to_node].emplace_back(roadPtr->roadId);
//
//                    double length = 0;
//                    int pt_count = pObject->nPntsSize;
//                    for(int j = 0; j < pt_count; ++j){
//                        double dx = pObject->pCoords[j * 2 + 0] / 3600.00;
//                        double dy = pObject->pCoords[j * 2 + 1] / 3600.00;
//
//                        if (j < pt_count - 1){
//                            double dx0 = pObject->pCoords[j * 2 + 2] / 3600.00;
//                            double dy0 = pObject->pCoords[j * 2 + 3] / 3600.00;
//                            length += CommonUtil::UT_GetMapDistance(dx, dy, dx0, dy0);
//                        }
//
//                        double new_dx, new_dy;
//                        coord_deoffset(dx, dy, &new_dx, &new_dy);
//
//                        shared_ptr<stPoint> ptPtr = make_shared<stPoint>(new_dx, new_dy);
//                        roadPtr->points.push_back(ptPtr);
//
//                        double utmx = new_dx * 3600;
//                        double utmy = new_dy * 3600;
//                        shared_ptr<stUTM> ptUTM = make_shared<stUTM>(utmx, utmy);
//                        roadPtr->utms.push_back(ptUTM);
//                    }
//                    roadPtr->length = int(length);
//
//                    g_meshRoads[strMeshId].emplace_back(roadPtr);
//                    g_meshRoadIds[strMeshId].insert(make_pair(roadPtr->roadId, roadPtr));
//                    g_meshBridgeRoads[strMeshId].emplace_back(roadPtr);
//                    if (group_bridges.empty() || group_bridges.empty()){
//                        vector<shared_ptr<stRoad>> group_road;
//                        done_roads.insert(strRoadId);
//                        group_road.emplace_back(roadPtr);
//                        group_bridges.insert(make_pair(group_index, group_road));
//                    }
//                }
//
//                if (strRoadClass == "41000" || strRoadClass == "42000" || strRoadClass == "43000"){
//                    if (formway != 1){
//                        MTL::SHPReleaseObject(pObject);
//                        continue;
//                    }
//                    // create quardTree spatial index
//                    geos::geom::CoordinateSequence *cl = new geos::geom::CoordinateArraySequence();
//
//                    shared_ptr<stRoad> roadPtr = make_shared<stRoad>();
//                    roadPtr->roadClass = strRoadClass;
//                    roadPtr->roadId = strRoadId;
//                    roadPtr->formway = formway;
//                    roadPtr->fromNode = from_node;
//                    roadPtr->toNode = to_node;
//
//                    double length = 0;
//                    int pt_count = pObject->nPntsSize;
//                    for(int j = 0; j < pt_count; ++j){
//                        double dx = pObject->pCoords[j * 2 + 0] / 3600.00;
//                        double dy = pObject->pCoords[j * 2 + 1] / 3600.00;
//
//                        if (j < pt_count - 1){
//                            double dx0 = pObject->pCoords[j * 2 + 2] / 3600.00;
//                            double dy0 = pObject->pCoords[j * 2 + 3] / 3600.00;
//                            length += CommonUtil::UT_GetMapDistance(dx, dy, dx0, dy0);
//                        }
//
//                        double new_dx, new_dy;
//                        coord_deoffset(dx, dy, &new_dx, &new_dy);
//                        cl->add(geos::geom::Coordinate(new_dx, new_dy));
//
//                        shared_ptr<stPoint> ptPtr = make_shared<stPoint>(new_dx, new_dy);
//                        roadPtr->points.push_back(ptPtr);
//
//                        double utmx = new_dx * 3600;
//                        double utmy = new_dy * 3600;
//                        shared_ptr<stUTM> ptUTM = make_shared<stUTM>(utmx, utmy);
//                        roadPtr->utms.push_back(ptUTM);
//                    }
//                    roadPtr->length = (int)length;
//
//                    fromNodes[from_node].emplace_back(roadPtr->roadId);
//                    toNodes[to_node].emplace_back(roadPtr->roadId);
//
//                    shared_ptr<geos::geom::LineString> line(gf->createLineString(cl));
//                    shared_ptr<geos::geom::Geometry> line_buffer(line->buffer(buffer));
//                    road_spatial_index->insert(line_buffer->getEnvelopeInternal(), roadPtr.get());
//
//                    g_meshRoads[strMeshId].emplace_back(roadPtr);
//                    g_meshRoadIds[strMeshId].insert(make_pair(roadPtr->roadId, roadPtr));
//                    g_meshMainRoads[strMeshId].emplace_back(roadPtr);
//                }
//
//                MTL::SHPReleaseObject(pObject);
//            }
//            g_meshSpatialIndexs.insert(make_pair(strMeshId, road_spatial_index));
//
//#if ADD_NEW_ROAD
//            // 串接Mesh内的道路
//            set<int> totalNodeIds;
//            for (const auto& from_it: fromNodes){
//                totalNodeIds.insert(from_it.first);
//            }
//            for (const auto& to_it: toNodes){
//                totalNodeIds.insert(to_it.first);
//            }
//
//            //  先把断开的拼接起来
//            for (auto& from_it: fromNodes) {
//                if (toNodes.find(from_it.first) == toNodes.end()) {
//                    // 找到一个开始节点
//                    vector<string> pRoadIds = from_it.second;
//                    string fromRoadId = pRoadIds[0];
//
//                    auto map_it = g_meshRoadIds.find(strMeshId);
//                    map<string, std::shared_ptr<stRoad>> pMap = map_it->second;
//                    auto road_iter = pMap.find(fromRoadId);
//                    shared_ptr<stRoad> pRoad = road_iter->second;
//
//                    double dx1 = pRoad->points[0]->x;
//                    double dy1 = pRoad->points[0]->y;
//
//                    double dx2 = pRoad->points[1]->x;
//                    double dy2 = pRoad->points[1]->y;
//
//                    // 查找起点附近的道路
//                    vector<void *> near_roads;
//                    shared_ptr<geos::geom::Point> start_point(gf->createPoint(geos::geom::Coordinate(dx1, dy1)));
//                    shared_ptr<geos::geom::Geometry> start_point_buffer(start_point->buffer(buffer));
//                    road_spatial_index->query(start_point_buffer->getEnvelopeInternal(), near_roads);
//
//                    double min_angle = 90;
//                    stRoad* pSelRoad = nullptr;
//                    double endx, endy;
//                    for (void *find_road: near_roads) {
//                        auto roadPtr = (stRoad *) find_road;
//
//                        if (roadPtr->roadId == pRoad->roadId){
//                            continue;
//                        }
//
//                        // 该道路的TNode能在FromNodes中找到的话，证明没有断开
//                        int road_to_id = roadPtr->toNode;
//                        if (fromNodes.find(road_to_id) != fromNodes.end()){
//                            continue;
//                        }
//
//                        double dx3 = roadPtr->points[roadPtr->points.size() - 2]->x;
//                        double dy3 = roadPtr->points[roadPtr->points.size() - 2]->y;
//
//                        double dx4 = roadPtr->points[roadPtr->points.size() - 1]->x;
//                        double dy4 = roadPtr->points[roadPtr->points.size() - 1]->y;
//
//                        endx = dx4;
//                        endy = dy4;
//
//                        // 计算起点与终点的距离
//                        double dis = CommonUtil::UT_GetMapDistance(dx1,dy1, dx4, dy4);
//                        if (dis > 50){
//                            continue;
//                        }
//
//                        // 计算角度
//                        double angle = CommonUtil::UT_GetLineAngle(dx1, dy1, dx2, dy2, dx3, dy3, dx4, dy4);
//
//                        if (angle > 10){
//                            continue;
//                        }
//
//                        if (angle < min_angle){
//                            min_angle = angle;
//                            pSelRoad = roadPtr;
//                        }
//                    }
//
//                    if (pSelRoad != nullptr){
//                        // 增加一条连接的road
//                        shared_ptr<stRoad> pNewRoad = make_shared<stRoad>();
//
//                        // 查找最大RoadId
//                        int max_roadId = 0;
//                        auto map_it1 = g_meshRoadIds.find(strMeshId);
//                        map<string, std::shared_ptr<stRoad>> pMap1 = map_it1->second;
//
//                        for (const auto& map_id: pMap1){
//                            istringstream strRoadId(map_id.first);
//                            int cur_roadId;
//                            strRoadId >> cur_roadId;
//
//                            if (cur_roadId > max_roadId){
//                                max_roadId = cur_roadId;
//                            }
//                        }
//                        max_roadId += 1;
//
//                        pNewRoad->roadId = std::to_string(max_roadId);
//                        pNewRoad->formway = pRoad->formway;
//                        pNewRoad->roadClass = pRoad->roadClass;
//                        pNewRoad->fromNode = pSelRoad->toNode;
//                        pNewRoad->toNode = pRoad->fromNode;
//
//                        shared_ptr<stPoint> ptPtr1 = make_shared<stPoint>(endx, endy);
//                        pNewRoad->points.push_back(ptPtr1);
//
//                        double utmx = endx * 3600;
//                        double utmy = endy * 3600;
//                        shared_ptr<stUTM> ptUTM = make_shared<stUTM>(utmx, utmy);
//                        pNewRoad->utms.push_back(ptUTM);
//
//                        shared_ptr<stPoint> ptPtr2 = make_shared<stPoint>(dx1, dy1);
//                        pNewRoad->points.push_back(ptPtr2);
//
//                        double utmx1 = dx1 * 3600;
//                        double utmy1 = dy1 * 3600;
//                        shared_ptr<stUTM> ptUTM1 = make_shared<stUTM>(utmx1, utmy1);
//                        pNewRoad->utms.push_back(ptUTM1);
//
//                        g_meshRoads[strMeshId].emplace_back(pNewRoad);
//                        g_meshRoadIds[strMeshId].insert(make_pair(pNewRoad->roadId, pNewRoad));
//
//                        fromNodes[pNewRoad->fromNode].emplace_back(pNewRoad->roadId);
//                        toNodes[pNewRoad->toNode].emplace_back(pNewRoad->roadId);
//                    }
//                }
//            }
//
//            for (auto& from_it: fromNodes){
//                if (toNodes.find(from_it.first) == toNodes.end()){
//                    // 找到一个开始节点
//                    vector<string> pRoadIds = from_it.second;
//
//                    bool isFirst = true;
//                    string firstRoad;
//                    if (!pRoadIds.empty() && pRoadIds.size() > 1){
//
//                        for (const auto& road_id: pRoadIds){
//                            // 跳过第一个
//                            if (isFirst){
//                                firstRoad = road_id;
//                                isFirst = false;
//                                continue;
//                            }
//                            int temp_index = 1;
//                            while(totalNodeIds.find(temp_index) != totalNodeIds.end()){
//                                temp_index++;
//                            }
//                            int new_from_id = temp_index;
//                            totalNodeIds.insert(new_from_id);
//
//                            fromNodes[new_from_id].emplace_back(road_id);
//
//                            auto map_it = g_meshRoadIds.find(strMeshId);
//                            map<string, std::shared_ptr<stRoad>> pMap = map_it->second;
//                            auto road_iter = pMap.find(road_id);
//                            shared_ptr<stRoad> pRoad = road_iter->second;
//
//                            pRoad->fromNode = new_from_id;
//
//                            from_it.second.clear();
//                            from_it.second.emplace_back(firstRoad);
//                        }
//                    }
//                }
//            }
//
//            for (auto& to_it: toNodes){
//                if (fromNodes.find(to_it.first) == fromNodes.end()){
//                    // 找到一个末尾节点
//                    vector<string> pRoadIds = to_it.second;
//
//                    bool isFirst = true;
//                    string firstRoad;
//                    if (!pRoadIds.empty() && pRoadIds.size() > 1){
//                        for (const auto& road_id: pRoadIds){
//                            if (isFirst){
//                                firstRoad = road_id;
//                                isFirst = false;
//                                continue;
//                            }
//
//                            int temp_index = 1;
//                            while(totalNodeIds.find(temp_index) != totalNodeIds.end()){
//                                temp_index++;
//                            }
//                            int new_to_id = temp_index;
//                            totalNodeIds.insert(new_to_id);
//
//                            toNodes[new_to_id].emplace_back(road_id);
//
//                            auto map_it = g_meshRoadIds.find(strMeshId);
//                            map<string, std::shared_ptr<stRoad>> pMap = map_it->second;
//                            auto road_iter = pMap.find(road_id);
//                            shared_ptr<stRoad> pRoad = road_iter->second;
//
//                            pRoad->toNode = new_to_id;
//                        }
//
//                        to_it.second.clear();
//                        to_it.second.emplace_back(firstRoad);
//                    }
//                }
//            }
//#endif
//            set<string> findHomeList;
//            for (const auto& from_it: fromNodes) {
//                if (toNodes.find(from_it.first) == toNodes.end()) {
//                    // 找到一个开始节点
//                    vector<string> pRoadIds = from_it.second;
//
//                    string headRoadId = pRoadIds[0];
//                    auto map_it = g_meshRoadIds.find(strMeshId);
//                    map<string, std::shared_ptr<stRoad>> pMap = map_it->second;
//                    auto road_iter = pMap.find(headRoadId);
//                    shared_ptr<stRoad> pRoad = road_iter->second;
//
//                    if (pRoad->formway != 1){
//                        // 开始节点只针对formway=1的主干路
//                        continue;
//                    }
//
//                    shared_ptr<roadList> pHeadList = make_shared<roadList>();
//                    pHeadList->roadId = headRoadId;
//                    pHeadList->pPrev = nullptr;
//
//                    int cur_to = pRoad->toNode;
//                    findHomeList.insert(pRoad->roadId);
//                    g_meshRoadGroups[strMeshId].emplace_back(pHeadList);
//
//                    shared_ptr<roadList> pList = pHeadList;
//
//                    double dx1 = pRoad->points[pRoad->points.size() - 2]->x;
//                    double dy1 = pRoad->points[pRoad->points.size() - 2]->y;
//
//                    double dx2 = pRoad->points[pRoad->points.size() - 1]->x;
//                    double dy2 = pRoad->points[pRoad->points.size() - 1]->y;
//
//                    auto to_it = fromNodes.find(cur_to);
//                    while (to_it != fromNodes.end()){
//                        vector<string> pNextRoadIds = to_it->second;
//
//                        shared_ptr<stRoad> pSelRoad = nullptr;
//                        double min_angle = 90;
//
//                        if (!pNextRoadIds.empty() && pNextRoadIds.size() == 1){
//                            string _roadId = pNextRoadIds[0];
//                            if (findHomeList.find(_roadId) != findHomeList.end()){
//                                break;
//                            }
//                            auto _map_it = g_meshRoadIds.find(strMeshId);
//                            map<string, std::shared_ptr<stRoad>> _pMap = _map_it->second;
//                            auto _road_iter = _pMap.find(_roadId);
//                            pSelRoad = _road_iter->second;
//
//                            if (pSelRoad->formway != 1){
//                                pSelRoad = nullptr;
//                                continue;
//                            }
//
//                            double dx3 = pSelRoad->points[0]->x;
//                            double dy3 = pSelRoad->points[0]->y;
//                            double dx4 = pSelRoad->points[1]->x;
//                            double dy4 = pSelRoad->points[1]->y;
//                            double angle = CommonUtil::UT_GetLineAngle(dx1, dy1, dx2, dy2, dx3, dy3, dx4, dy4);
//
//                            if (angle > 150){
//                                // 对向两条车道连在了一起
//                                shared_ptr<roadList> pNewHead = make_shared<roadList>();
//                                pNewHead->roadId = pSelRoad->roadId;
//                                findHomeList.insert(pSelRoad->roadId);
//                                g_meshRoadGroups[strMeshId].emplace_back(pNewHead);
//                                cur_to = pSelRoad->toNode;
//                                pRoad = pSelRoad;
//                                dx1 = pRoad->points[pRoad->points.size() - 2]->x;
//                                dy1 = pRoad->points[pRoad->points.size() - 2]->y;
//                                dx2 = pRoad->points[pRoad->points.size() - 1]->x;
//                                dy2 = pRoad->points[pRoad->points.size() - 1]->y;
//
//                                to_it = fromNodes.find(cur_to);
//                                pList = pNewHead;
//
//                                continue;
//                            }
//                        }else{
//                            /* 比较下一条道路的选择，
//                             * 比较原则：
//                             * 1. 角度最小的那个
//                             * 2. 角度也要小于10
//                             */
//
//                            for (const auto& propal_id: pNextRoadIds){
//                                if (findHomeList.find(propal_id) != findHomeList.end()){
//                                    continue;
//                                }
//                                auto _map_it = g_meshRoadIds.find(strMeshId);
//                                map<string, std::shared_ptr<stRoad>> _pMap = _map_it->second;
//                                auto _road_iter = _pMap.find(propal_id);
//                                shared_ptr<stRoad> pPropRoad = _road_iter->second;
//
//                                if (pPropRoad->formway != 1){
//                                    continue;
//                                }
//
//                                double dx3 = pPropRoad->points[0]->x;
//                                double dy3 = pPropRoad->points[0]->y;
//                                double dx4 = pPropRoad->points[1]->x;
//                                double dy4 = pPropRoad->points[1]->y;
//
//                                if (pPropRoad->formway == 1){
//                                    pSelRoad = pPropRoad;
//                                    break;
//                                }
//
//                                double angle = CommonUtil::UT_GetLineAngle(dx1, dy1, dx2, dy2, dx3, dy3, dx4, dy4);
//
//                                if (angle < min_angle){
//                                    min_angle = angle;
//                                    pSelRoad = pPropRoad;
//                                }
//                            }
//                        }
//
//                        if (pSelRoad == nullptr){
//                            break;
//                        }
//
//                        string nextRoadId = pSelRoad->roadId;
//                        findHomeList.insert(nextRoadId);
//                        shared_ptr<roadList> pCurList = make_shared<roadList>();
//                        pCurList->roadId = nextRoadId;
//                        pCurList->pPrev = pList;
//                        pList->pNext = pCurList;
//                        cur_to = pSelRoad->toNode;
//                        to_it = fromNodes.find(cur_to);
//                        pList = pCurList;
//
//                        auto road_iter1 = pMap.find(nextRoadId);
//                        shared_ptr<stRoad> pRoad1 = road_iter1->second;
//                        dx1 = pRoad1->points[pRoad1->points.size() - 2]->x;
//                        dy1 = pRoad1->points[pRoad1->points.size() - 2]->y;
//                        dx2 = pRoad1->points[pRoad1->points.size() - 1]->x;
//                        dy2 = pRoad1->points[pRoad1->points.size() - 1]->y;
//                    }
//                }
//            }
//
//#if ADD_NEW_ROAD
//            // 计算立交桥的候选任务框
//            set<int> totalBridgeNodeIds;
//            for (const auto& from_it: fromBridgeNodes){
//                totalBridgeNodeIds.insert(from_it.first);
//            }
//            for (const auto& to_it: toBridgeNodes){
//                totalBridgeNodeIds.insert(to_it.first);
//            }
//
//            /*
//             * 先将共点的线段分开
//             */
//            for (auto& from_bridge_it: fromBridgeNodes){
//                vector<string> pRoadIds = from_bridge_it.second;
//
//                bool isFirst = true;
//                string firstRoad;
//                if (!pRoadIds.empty() && pRoadIds.size() > 1){
//
//                    for (const auto& road_id: pRoadIds){
//                        // 跳过第一个
//                        if (isFirst){
//                            firstRoad = road_id;
//                            isFirst = false;
//                            continue;
//                        }
//                        int temp_index = 1;
//                        while(totalBridgeNodeIds.find(temp_index) != totalBridgeNodeIds.end()){
//                            temp_index++;
//                        }
//                        int new_from_id = temp_index;
//                        totalBridgeNodeIds.insert(new_from_id);
//
//                        fromBridgeNodes[new_from_id].emplace_back(road_id);
//
//                        auto map_it = g_meshRoadIds.find(strMeshId);
//                        map<string, std::shared_ptr<stRoad>> pMap = map_it->second;
//                        auto road_iter = pMap.find(road_id);
//                        shared_ptr<stRoad> pRoad = road_iter->second;
//
//                        pRoad->fromNode = new_from_id;
//                    }
//
//                    from_bridge_it.second.clear();
//                    from_bridge_it.second.emplace_back(firstRoad);
//                }
//            }
//
//            for (auto& to_bridge_it: toBridgeNodes){
//                // 找到一个末尾节点
//                vector<string> pRoadIds = to_bridge_it.second;
//
//                bool isFirst = true;
//                string firstRoad;
//                if (!pRoadIds.empty() && pRoadIds.size() > 1){
//                    for (const auto& road_id: pRoadIds){
//                        if (isFirst){
//                            firstRoad = firstRoad = road_id;
//                            isFirst = false;
//                            continue;
//                        }
//
//                        int temp_index = 1;
//                        while(totalBridgeNodeIds.find(temp_index) != totalBridgeNodeIds.end()){
//                            temp_index++;
//                        }
//                        int new_to_id = temp_index;
//                        totalBridgeNodeIds.insert(new_to_id);
//
//                        toBridgeNodes[new_to_id].emplace_back(road_id);
//
//                        auto map_it = g_meshRoadIds.find(strMeshId);
//                        map<string, std::shared_ptr<stRoad>> pMap = map_it->second;
//                        auto road_iter = pMap.find(road_id);
//                        shared_ptr<stRoad> pRoad = road_iter->second;
//
//                        pRoad->toNode = new_to_id;
//                    }
//                    to_bridge_it.second.clear();
//                    to_bridge_it.second.emplace_back(firstRoad);
//                }
//            }
//
//            for (const auto& from_bridge_it: fromBridgeNodes) {
//                if (toBridgeNodes.find(from_bridge_it.first) == toBridgeNodes.end()) {
//                    // 找到一个开始节点
//                    vector<string> pRoadIds = from_bridge_it.second;
//
//                    string headRoadId = pRoadIds[0];
//                    auto map_it = g_meshRoadIds.find(strMeshId);
//                    map<string, std::shared_ptr<stRoad>> pMap = map_it->second;
//                    auto road_iter = pMap.find(headRoadId);
//                    shared_ptr<stRoad> pRoad = road_iter->second;
//
//                    double dx1 = pRoad->points[pRoad->points.size() - 2]->x;
//                    double dy1 = pRoad->points[pRoad->points.size() - 2]->y;
//
//                    double dx2 = pRoad->points[pRoad->points.size() - 1]->x;
//                    double dy2 = pRoad->points[pRoad->points.size() - 1]->y;
//
//                    shared_ptr<roadList> pHeadList = make_shared<roadList>();
//                    pHeadList->roadId = headRoadId;
//                    pHeadList->pPrev = nullptr;
//
//                    int cur_to = pRoad->toNode;
//                    g_meshBridgeGroups[strMeshId].emplace_back(pHeadList);
//
//                    shared_ptr<roadList> pList = pHeadList;
//
//                    auto to_bridge_it = fromBridgeNodes.find(cur_to);
//                    while (to_bridge_it != fromBridgeNodes.end()
//                           && !to_bridge_it->second.empty()
//                           && to_bridge_it->second.size() > 0){
//                        vector<string> pNextRoadIds = to_bridge_it->second;
//
//                        string _roadId = pNextRoadIds[0];
//                        auto _map_it = g_meshRoadIds.find(strMeshId);
//                        map<string, std::shared_ptr<stRoad>> _pMap = _map_it->second;
//                        auto _road_iter = _pMap.find(_roadId);
//                        shared_ptr<stRoad> pSelRoad = _road_iter->second;
//
//                        double dx3 = pSelRoad->points[0]->x;
//                        double dy3 = pSelRoad->points[0]->y;
//                        double dx4 = pSelRoad->points[1]->x;
//                        double dy4 = pSelRoad->points[1]->y;
//                        double angle = CommonUtil::UT_GetLineAngle(dx1, dy1, dx2, dy2, dx3, dy3, dx4, dy4);
//
//                        if (angle > 150){
//                            // 对向两条车道连在了一起
//                            shared_ptr<roadList> pNewHead = make_shared<roadList>();
//                            pNewHead->roadId = pSelRoad->roadId;
//                            g_meshBridgeGroups[strMeshId].emplace_back(pNewHead);
//                            cur_to = pSelRoad->toNode;
//                            pRoad = pSelRoad;
//                            dx1 = pRoad->points[pRoad->points.size() - 2]->x;
//                            dy1 = pRoad->points[pRoad->points.size() - 2]->y;
//                            dx2 = pRoad->points[pRoad->points.size() - 1]->x;
//                            dy2 = pRoad->points[pRoad->points.size() - 1]->y;
//
//                            to_bridge_it = fromBridgeNodes.find(cur_to);
//                            pList = pNewHead;
//
//                            continue;
//                        }
//
//                        string nextRoadId = pSelRoad->roadId;
//                        shared_ptr<roadList> pCurList = make_shared<roadList>();
//                        pCurList->roadId = nextRoadId;
//                        pCurList->pPrev = pList;
//                        pList->pNext = pCurList;
//                        cur_to = pSelRoad->toNode;
//                        to_bridge_it = fromBridgeNodes.find(cur_to);
//                        pList = pCurList;
//
//                        auto road_iter1 = pMap.find(nextRoadId);
//                        shared_ptr<stRoad> pRoad1 = road_iter1->second;
//                        dx1 = pRoad1->points[pRoad1->points.size() - 2]->x;
//                        dy1 = pRoad1->points[pRoad1->points.size() - 2]->y;
//                        dx2 = pRoad1->points[pRoad1->points.size() - 1]->x;
//                        dy2 = pRoad1->points[pRoad1->points.size() - 1]->y;
//                    }
//                }
//            }
//#else
//            const auto& bridgeIter = g_meshBridgeRoads.find(strMeshId);
//            if (bridgeIter != g_meshBridgeRoads.end()){
//                for (const auto& pBridgeRoad: bridgeIter->second){
//                    if (pBridgeRoad->length <= 1)
//                        continue;
//                    shared_ptr<roadList> pHeadList = make_shared<roadList>();
//                    pHeadList->roadId = pBridgeRoad->roadId;
//                    pHeadList->pPrev = nullptr;
//                    pHeadList->pNext = nullptr;
//                    g_meshBridgeGroups[strMeshId].emplace_back(pHeadList);
//                }
//            }
//#endif
//            // 生成立交桥的候选任务框
//            auto bridge_group = g_meshBridgeGroups.find(strMeshId);
//            if (bridge_group != g_meshBridgeGroups.end()){
//                vector<shared_ptr<roadList>> bridge_lists = bridge_group->second;
//                int bridge_group_index = 0;
//                for (const auto& check_it: bridge_lists){
//                    bridge_group_index++;
//
//                    shared_ptr<roadList> pList = check_it;
//                    vector<shared_ptr<stRoad>> roadGroup;
//                    while(pList){
//                        auto _map_it = g_meshRoadIds.find(strMeshId);
//                        map<string, std::shared_ptr<stRoad>> _pMap = _map_it->second;
//                        auto _road_iter = _pMap.find(pList->roadId);
//                        shared_ptr<stRoad> pRoad = _road_iter->second;
//
//                        if (pRoad->length >= 1){
//                            roadGroup.emplace_back(pRoad);
//                        }
//
//                        pList = pList->pNext;
//                    }
//                    g_meshBridgeProposals[strMeshId].insert(make_pair(bridge_group_index, roadGroup));
//                }
//            }
//
//            // 计算立交桥的范围
//            auto bridge_iter = g_meshBridgeRoads.find(strMeshId);
//            if (bridge_iter != g_meshBridgeRoads.end()){
//                vector<shared_ptr<stRoad>> pRoads = bridge_iter->second;
//
//                string last_key;
//                vector<shared_ptr<stRoad>> last_group;
//                while (done_roads.size() != pRoads.size()){
//                    bool loopHasHome = false;
//
//                    for (const auto& road_it: pRoads){
//                        double min_dis = 100000;
//                        bool hasHome = false;
//
//                        if (done_roads.find(road_it->roadId) != done_roads.end()){
//                            continue;
//                        }
//
//                        double dx1 = road_it->points[0]->x;
//                        double dy1 = road_it->points[0]->y;
//                        double dx2 = road_it->points[road_it->points.size() - 1]->x;
//                        double dy2 = road_it->points[road_it->points.size() - 1]->y;
//
//                        for (auto& home_it: group_bridges){
//                            for (auto& home_road: home_it.second){
//                                if (home_road->roadId == road_it->roadId){
//                                    continue;
//                                }
//
//                                double dx3 = home_road->points[0]->x;
//                                double dy3 = home_road->points[0]->y;
//                                double dx4 = home_road->points[home_road->points.size() - 1]->x;
//                                double dy4 = home_road->points[home_road->points.size() - 1]->y;
//
//                                // A起点 B终点
//                                double dis = CommonUtil::UT_GetMapDistance(dx1, dy1, dx4, dy4);
//                                if (dis < min_dis){
//                                    min_dis = dis;
//                                }
//
//                                if (min_dis < 150.00){
//                                    done_roads.insert(road_it->roadId);
//                                    group_bridges[home_it.first].emplace_back(road_it);
//                                    hasHome = true;
//                                    loopHasHome = true;
//                                    break;
//                                }
//
//                                // A起点 B起点
//                                dis = CommonUtil::UT_GetMapDistance(dx1, dy1, dx3, dy3);
//                                if (dis < min_dis){
//                                    min_dis = dis;
//                                }
//
//                                if (min_dis < 150.00){
//                                    done_roads.insert(road_it->roadId);
//                                    group_bridges[home_it.first].emplace_back(road_it);
//                                    hasHome = true;
//                                    loopHasHome = true;
//                                    break;
//                                }
//
//                                // A终点 B起点
//                                dis = CommonUtil::UT_GetMapDistance(dx2, dy2, dx3, dy3);
//                                if (dis < min_dis){
//                                    min_dis = dis;
//                                }
//
//                                if (min_dis < 150.00){
//                                    done_roads.insert(road_it->roadId);
//                                    group_bridges[home_it.first].emplace_back(road_it);
//                                    hasHome = true;
//                                    loopHasHome = true;
//                                    break;
//                                }
//
//                                // A终点 B终点
//                                dis = CommonUtil::UT_GetMapDistance(dx2, dy2, dx4, dy4);
//                                if (dis < min_dis){
//                                    min_dis = dis;
//                                }
//
//                                if (min_dis < 150.00){
//                                    done_roads.insert(road_it->roadId);
//                                    group_bridges[home_it.first].emplace_back(road_it);
//                                    hasHome = true;
//                                    loopHasHome = true;
//                                    break;
//                                }
//                            }
//
//                            if (hasHome){
//                                break;
//                            }
//                        }
//
//                        if (!hasHome){
//                            last_key = road_it->roadId;
//                            last_group.clear();
//                            last_group.emplace_back(road_it);
//                        }
//                    }
//
//                    if (!loopHasHome){
//                        group_index += 1;
//                        done_roads.insert(last_key);
//                        group_bridges.insert(make_pair(group_index, last_group));
//                    }
//                }
//
//                map<int, vector<double> > bridge_reGroup;
//                for (const auto& group_it : group_bridges){
//                    double dxmin = 180;
//                    double dymin = 90;
//                    double dxmax = -180;
//                    double dymax = -90;
//                    for (const auto& group_road: group_it.second){
//                        for (const auto& road_pt_it: group_road->points){
//                            double x1 = road_pt_it->x;
//                            double y1 = road_pt_it->y;
//
//                            if (x1 < dxmin)
//                                dxmin = x1;
//
//                            if (x1 > dxmax)
//                                dxmax = x1;
//
//                            if (y1 < dymin)
//                                dymin = y1;
//
//                            if (y1 > dymax)
//                                dymax = y1;
//                        }
//                    }
//
//                    // 四边外扩50m
//                    double xstep = CommonUtil::UT_GetMapDistance(dxmin, dymin, dxmin + 0.1, dymin);
//                    double ystep = CommonUtil::UT_GetMapDistance(dxmin, dymin, dxmin, dymin + 0.1);
//
//                    double new_xmin = dxmin - 50.00 / xstep * 0.1;
//                    double new_xmax = dxmax + 50.00 / xstep * 0.1;
//
//                    double new_ymin = dymin - 50.00 / ystep * 0.1;
//                    double new_ymax = dymax + 50.00 / ystep * 0.1;
//
//                    vector<double> boundary;
//                    boundary.emplace_back(new_xmin);
//                    boundary.emplace_back(new_ymin);
//                    boundary.emplace_back(new_xmax);
//                    boundary.emplace_back(new_ymax);
//                    bridge_reGroup.insert(make_pair(group_it.first, boundary));
//                }
//
//                // 合并有交叉的桥
//                map<int, set<int>> del_bridges;
//                for (const auto& bridge_it1: bridge_reGroup){
//                    if (del_bridges.find(bridge_it1.first) != del_bridges.end()){
//                        continue;
//                    }
//                    del_bridges[bridge_it1.first].insert(bridge_it1.first);
//
//                    vector<double> boundary1 = bridge_it1.second;
//                    double xmin1 = boundary1[0];
//                    double ymin1 = boundary1[1];
//                    double xmax1 = boundary1[2];
//                    double ymax1 = boundary1[3];
//
//                    for (const auto& bridge_it2: bridge_reGroup){
//                        if (bridge_it1.first == bridge_it2.first){
//                            continue;
//                        }
//
//                        vector<double> boundary2 = bridge_it2.second;
//                        double xmin2 = boundary2[0];
//                        double ymin2 = boundary2[1];
//                        double xmax2 = boundary2[2];
//                        double ymax2 = boundary2[3];
//
//                        bool hasHome = false;
//                        if (!hasHome){
//                            if ((xmin1 < xmin2 && xmin2 < xmax1 && ymin1 < ymin2 && ymin2 < ymax1)){
//                                hasHome = true;
//                            }
//                        }
//                        if (!hasHome){
//                            if ((xmin1 < xmin2 && xmin2 < xmax1 && ymin1 < ymax2 && ymax2 < ymax1)){
//                                hasHome = true;
//                            }
//                        }
//                        if (!hasHome){
//                            if ((xmin1 < xmax2 && xmax2 < xmax1 && ymin1 < ymax2 && ymax2 < ymax1)){
//                                hasHome = true;
//                            }
//                        }
//                        if (!hasHome){
//                            if ((xmin1 < xmax2 && xmax2 < xmax1 && ymin1 < ymin2 && ymin2 < ymax1)){
//                                hasHome = true;
//                            }
//                        }
//
//                        if (!hasHome){
//                            if ((xmin2 < xmin1 && xmin1 < xmax2 && ymin2 < ymin1 && ymin1 < ymax2)){
//                                hasHome = true;
//                            }
//                        }
//                        if (!hasHome){
//                            if ((xmin2 < xmin1 && xmin1 < xmax2 && ymin2 < ymax1 && ymax1 < ymax2)){
//                                hasHome = true;
//                            }
//                        }
//                        if (!hasHome){
//                            if ((xmin2 < xmax1 && xmax1 < xmax2 && ymin2 < ymax1 && ymax1 < ymax2)){
//                                hasHome = true;
//                            }
//                        }
//                        if (!hasHome){
//                            if ((xmin2 < xmax1 && xmax1 < xmax2 && ymin2 < ymin1 && ymin1 < ymax2)){
//                                hasHome = true;
//                            }
//                        }
//
//                        if (hasHome) {
//                            del_bridges[bridge_it1.first].insert(bridge_it2.first);
//                        }
//                    }
//                }
//
//                map<int, set<int> > realGroup;
//                int reGroup_index = 0;
//                int last_index = 0;
//                set<int> done_bridges;
//                while (done_bridges.size() != del_bridges.size()){
//                    bool loopHasHome = false;
//
//                    for (const auto& bridge_it: del_bridges){
//                        bool hasHome = false;
//
//                        if (done_bridges.find(bridge_it.first) != done_bridges.end()){
//                            continue;
//                        }
//
//                        for (auto& home_it: realGroup){
//                            if (home_it.second.find(bridge_it.first) != home_it.second.end()){
//                                for (const auto& new_index: bridge_it.second){
//                                    home_it.second.insert(new_index);
//                                }
//                                done_bridges.insert(bridge_it.first);
//                                hasHome = true;
//                                loopHasHome = true;
//                                break;
//                            }
//                        }
//
//                        if (!hasHome){
//                            last_index = bridge_it.first;
//                        }
//                    }
//
//                    if (!loopHasHome){
//                        reGroup_index += 1;
//                        done_bridges.insert(last_index);
//                        const auto new_it = del_bridges.find(last_index);
//                        for(const auto& new_id: new_it->second){
//                            realGroup[reGroup_index].insert(new_id);
//                        }
//                        last_index = 0;
//                    }
//                }
//
//                for (const auto& group_it: realGroup){
//                    double dxmin = 180;
//                    double dymin = 90;
//                    double dxmax = -180;
//                    double dymax = -90;
//                    for (const auto& group_new_index: group_it.second){
//                        const auto reGroup_it = bridge_reGroup.find(group_new_index);
//                        vector<double> boundary = reGroup_it->second;
//
//                        if (dxmin > boundary[0]){
//                            dxmin = boundary[0];
//                        }
//                        if (dymin > boundary[1]){
//                            dymin = boundary[1];
//                        }
//                        if (dxmax < boundary[2]){
//                            dxmax = boundary[2];
//                        }
//                        if (dymax < boundary[3]){
//                            dymax = boundary[3];
//                        }
//                    }
//
//                    vector<double> boundary;
//                    boundary.emplace_back(dxmin);
//                    boundary.emplace_back(dymin);
//                    boundary.emplace_back(dxmax);
//                    boundary.emplace_back(dymax);
//                    g_meshBridgeBoundarys[strMeshId].emplace_back(boundary);
//                }
//            }
//
//            // 重新划分立交桥匝道的归属
//            vector<shared_ptr<stRoad>> totalRamps;
//            for (const auto& bridgeIter: g_meshBridgeProposals[strMeshId]){
//                for (const auto& road: bridgeIter.second){
//                    totalRamps.emplace_back(road);
//                }
//            }
//            g_meshBridgeProposals[strMeshId].clear();
//
//            for (const auto& road: totalRamps){
//                int inBridgeIndex = -1;
//                this->isPointInBridge(strMeshId, road->points[0]->x, road->points[0]->y, inBridgeIndex);
//
//                g_meshBridgeProposals[strMeshId][inBridgeIndex].emplace_back(road);
//            }
//
//            // 计算主干路的任务区域
//            auto road_group = g_meshRoadGroups.find(strMeshId);
//            if (road_group == g_meshRoadGroups.end()){
//                return true;
//            }
//
//            vector<shared_ptr<roadList>> lists = road_group->second;
//            int task_group_index = 0;
//            for (const auto& check_it: lists){
//                task_group_index++;
//
//                int group_length = 0;
//
//                shared_ptr<roadList> pList = check_it;
//                vector<shared_ptr<stRoad>> roadGroup;
//                while(pList){
//
//                    auto _map_it = g_meshRoadIds.find(strMeshId);
//                    map<string, std::shared_ptr<stRoad>> _pMap = _map_it->second;
//                    auto _road_iter = _pMap.find(pList->roadId);
//                    shared_ptr<stRoad> pRoad = _road_iter->second;
//
//                    group_length += pRoad->length;
//                    roadGroup.emplace_back(pRoad);
//
//                    if (group_length >= 7000) {
//                        g_meshProposals[strMeshId].insert(make_pair(task_group_index, roadGroup));
//                        roadGroup.clear();
//                        group_length = 0;
//                        task_group_index++;
//                        /*
//                        double dx = pRoad->points[pRoad->points.size() - 1]->x;
//                        double dy = pRoad->points[pRoad->points.size() - 1]->y;
//                        int inBridgeIndex = -1;
//                        bool isInBridge = this->isPointInBridge(strMeshId, dx, dy, inBridgeIndex);
//                        if (!pList->pNext || !isInBridge){
//
//                        }
//                        */
//                    }
//
//                    pList = pList->pNext;
//                }
//                if (!roadGroup.empty()){
//                    if (roadGroup.size() > 1){
//                        g_meshProposals[strMeshId].insert(make_pair(task_group_index, roadGroup));
//                        roadGroup.clear();
//                        group_length = 0;
//                    }
//                    else{
//                        g_meshProposals[strMeshId][task_group_index].emplace_back(roadGroup[0]);
//                        roadGroup.clear();
//                        group_length = 0;
//                    }
//                }
//            }
//
//            // 起名字
//            for (const auto& meshIter: g_meshProposals){
//                string strMeshId = meshIter.first;
//
//                for(const auto& rangeIter: meshIter.second){
//                    int rangeIndex = rangeIter.first;
//                    string strRangeName = cityCode_;
//                    strRangeName.append("_");
//                    strRangeName.append(strMeshId);
//                    strRangeName.append("_Main_");
//                    strRangeName.append(std::to_string(rangeIndex));
//
//                    g_meshProposalNames[strMeshId].insert(make_pair(rangeIndex, strRangeName));
//                }
//            }
//            for (const auto& meshIter: g_meshBridgeProposals){
//                string strMeshId = meshIter.first;
//
//                for(const auto& rangeIter: meshIter.second){
//                    int rangeIndex = rangeIter.first;
//                    string strRangeName = cityCode_;
//                    strRangeName.append("_");
//                    strRangeName.append(strMeshId);
//                    strRangeName.append("_Bridge_");
//                    strRangeName.append(std::to_string(rangeIndex));
//
//                    g_meshBridgeProposalNames[strMeshId].insert(make_pair(rangeIndex, strRangeName));
//                }
//            }

            return true;
        }

        void CMesh::getMeshBoundary(double *coordinates, const std::string& meshId) const {
            const auto& mesh_iter = g_kvMesh.find(meshId);
            if (mesh_iter == g_kvMesh.end()){
                return;
            }
            std::shared_ptr<MeshRectangle> pRect = mesh_iter->second;
            coordinates[0] = pRect->xmin;
            coordinates[1] = pRect->ymin;
            coordinates[2] = pRect->xmax;
            coordinates[3] = pRect->ymax;
        }

        void CMesh::getCalcMeshBoundary(double *coordinates, const std::string& meshId) const {
            const auto& mesh_iter = g_kvCalcMesh.find(meshId);
            if (mesh_iter == g_kvCalcMesh.end()){
                return;
            }
            std::shared_ptr<MeshRectangle> pRect = mesh_iter->second;
            coordinates[0] = pRect->xmin;
            coordinates[1] = pRect->ymin;
            coordinates[2] = pRect->xmax;
            coordinates[3] = pRect->ymax;
        }

        void CMesh::getAllMesh(vector<string>& meshList) {
            for (const auto& mesh_it:g_kvMesh){
                meshList.push_back(mesh_it.first);
            }
        }

//        void CMesh::WriteToDBF(const char *pAttr, MTL::DBFHandle *pDBF, long nNum) {
//            map<int, int> typeCol;
//            typeCol.insert(make_pair<int, int>(0, 0));
//
//            std::string s(pAttr);
//            char cTemp[512] = {0};
//            int nPos = 0, nTemp = 0, nCol = 0;
//            //解析属性字符串
//            while (nTemp < s.length()) {
//                nPos = static_cast<int>(s.find("\t", nTemp));
//                if (nPos == -1) {
//                    nPos = static_cast<int>(s.length());
//                }
//
//                if (nPos != nTemp) {
//                    sprintf(cTemp, "%s", s.substr(nTemp, nPos - nTemp).c_str());
//                    int nDbfType = 0;
//                    if (nCol == typeCol.size()) {
//                        break;
//                    }
//                    auto it = typeCol.find(nCol);
//                    nDbfType = it->second;
//
//                    if (nDbfType == 0) {
//                        MTL::DBFWriteStringAttribute(pDBF, nNum, nCol, cTemp);
//                    } else if (nDbfType == 1) {
//                        MTL::DBFWriteIntegerAttribute(pDBF, nNum, nCol, atoi(cTemp));
//                    } else if (nDbfType == 2) {
//                        MTL::DBFWriteDoubleAttribute(pDBF, nNum, nCol, atof(cTemp));
//                    }
//                }
//                nTemp = nPos + 1;
//                nCol++;
//            }
//        }

        bool CMesh::isPointInBridge(const string& strMeshId, const double& dx, const double& dy, int& boundary_index) {
            bool isBridge = false;

            int cur_index = 0;
            for (const auto& bridge_it: g_meshBridgeBoundarys){
                if (!strMeshId.empty() || !strMeshId.empty()){
                    if (bridge_it.first != strMeshId){
                        continue;
                    }
                }
                for (const auto& boundary_it: bridge_it.second){
                    double xmin = boundary_it[0];
                    double ymin = boundary_it[1];
                    double xmax = boundary_it[2];
                    double ymax = boundary_it[3];

                    if (xmin < dx && xmax > dx && ymin < dy && ymax > dy){
                        isBridge = true;
                        break;
                    }
                    cur_index++;
                }

                if (isBridge){
                    boundary_index = cur_index;
                    break;
                }
            }

            return isBridge;
        }

        bool CMesh::isLineInBridge(const string& strMeshId, const double *points, const int &pt_count, int& boundary_index) {
            bool isBridge = true;

            for (int i=0; i<pt_count; i++){
                double dx = points[i * 2 + 0];
                double dy = points[i * 2 + 1];

                if (!isPointInBridge(strMeshId, dx, dy, boundary_index)){
                    isBridge = false;
                    break;
                }
            }

            return isBridge;
        }

        bool CMesh::isLineIntersectBridge(const string& strMeshId, const double *points, const int &pt_count, vector<int>& boundary_indexes) {
            bool isBridge = false;
            boundary_indexes.clear();

            set<int> bridge_indexes;
            for (int i=0; i<pt_count; i++){
                double dx = points[i * 2 + 0];
                double dy = points[i * 2 + 1];

                int boundary_index = -1;
                if (isPointInBridge(strMeshId, dx, dy, boundary_index)){
                    isBridge = true;
                    bridge_indexes.insert(boundary_index);
                }
            }

            if (isBridge){
                for (const auto& bridge_id: bridge_indexes){
                    boundary_indexes.emplace_back(bridge_id);
                }
            }

            return isBridge;
        }

        void CMesh::getBridgeBoundary(const string& strMesh, const int &bridge_index, double *boundary) {
            if (strMesh.empty() || strMesh.empty()){
                return;
            }

            if (bridge_index == -1){
                return;
            }

            auto bridge_iter = g_meshBridgeBoundarys.find(strMesh);
            auto boundarys = bridge_iter->second;
            if (bridge_index >= boundarys.size()){
                return;
            }
            auto boundary_it = boundarys[bridge_index];

            int i = 0;
            for (const auto& coord: boundary_it){
                boundary[i] = coord;
                i++;
            }
        }

        void CMesh::getBridgeCount(const string &strMesh, int &bridge_count) {
            if (strMesh.empty() || strMesh.empty()){
                bridge_count = 0;
                return;
            }

            auto bridge_iter = g_meshBridgeBoundarys.find(strMesh);
            bridge_count = (int)(bridge_iter->second.size());
        }
    }
}

