#ifndef FULL_AUTOMAPPING_CMESH_H
#define FULL_AUTOMAPPING_CMESH_H

#include <string>
#include <vector>
#include <map>
#include <set>
#include <memory>
#include <stdio.h>
#include <fstream>

#include <geos/indexQuadtree.h>
#include <Poco/Logger.h>

#include "MeshGridObjectExt.hpp"
//#include "util/shplib.h"

using namespace kd::autohdmap;
using Poco::Logger;

namespace kd{
    namespace autohdmap{
        struct roadList{
            shared_ptr<roadList> pPrev;
            shared_ptr<roadList> pNext;
            string roadId;

            roadList(){
                roadId = "";
                pPrev = nullptr;
                pNext = nullptr;
            }
        };

        struct stPoint{
            double x;
            double y;
            stPoint(double x1, double y1){
                x = x1;
                y = y1;
            }
        };
        struct stUTM{
            double x;
            double y;
            stUTM(double x1, double y1){
                x = x1;
                y = y1;
            }
        };
        struct stRoad{
            std::string roadClass;
            std::string roadId;
            int formway;
            int fromNode;
            int toNode;
            int length; // 长度：单位米
            vector<shared_ptr<stPoint>> points;
            vector<shared_ptr<stUTM> > utms;
            stRoad(){
                roadClass = roadId = "";
                formway = fromNode = toNode = length = 0;
            }
            ~stRoad(){
                for (auto& pt:points){
                    pt.reset();
                }
                points.clear();
                for (auto& pt:utms){
                    pt.reset();
                }
                utms.clear();
            }
        };
        class CMesh {

        public:
            CMesh();
            virtual ~CMesh();

        public:
            static CMesh* getInstance();

            bool initMeshByList(const int& meshLevel, const vector<string>& meshList);

            bool saveMeshAsShape(const vector<string>& meshList, const string& fileName);
            bool saveBridgeAsShape(const string& fileName);

            bool saveMeshAsGeoJson(const vector<string>& meshList, const string& fileName);
            bool saveBridgeAsGeoJson(const string& fileName);

            void setCityRegion(const string& dataDir, const string& cityCode,
                    const double& dxmin, const double& dymin,
                    const double& dxmax, const double& dymax);

            void setCityInfo(const string& dataDir, const string& cityCode);

            /**
             * 根据经纬度获取所在mesh
             * @longitude 经度
             * @latitude 纬度
             * @return meshId
             */
            std::string getMeshId(const double& longitude, const double& latitude) const ;


            /*
             * 传入多个点，一般为线上的形状点，返回跨越的imageType多个meshId
             * @params:coordinates，形状点坐标，x,y,x,y
             * @params:ptCount，形状点个数
             * @params:meshList，线段跨越的meshId
             * @return:
             */
            void getMeshList(const double *coordinates, const int& ptCount, std::vector<std::string>& meshList) const;

            /*
             * mesh对应的bbox
             * @params:coordinates，点坐标，x,y,x,y
             * @return:
             */
            void getMeshBoundary(double* coordinates, const std::string& meshId) const;
            void getCalcMeshBoundary(double* coordinates, const std::string& meshId) const;

            /*
             * 获取记录的所有Mesh
             */
            void getAllMesh(vector<string>& meshList);

            /*
             * 判断点是否在立交桥的范围
             */
            bool isPointInBridge(const string& strMeshId, const double& dx, const double& dy, int& boundary_index);

            /*
             * 判断线是否在立交桥的范围
             */
            bool isLineInBridge(const string& strMeshId, const double* points, const int& pt_count, int& boundary_index);

            bool isLineIntersectBridge(const string& strMeshId, const double* points, const int& pt_count, vector<int>& boundary_indexes);

            void getBridgeCount(const string& strMesh, int& bridge_count);
            void getBridgeBoundary(const string& strMesh, const int& bridge_index, double* boundary);


        private:
            bool initMeshRoad(const char *filePath, const char *mesh_name);
            //void WriteToDBF(const char *pAttr, MTL::DBFHandle *pDBF, long nNum);

        private:
            double dxmin_;
            double dymin_;
            double dxmax_;
            double dymax_;
            string dataDirectory_;
            string cityCode_;
            std::map<std::string, std::shared_ptr<MeshRectangle>> g_kvMesh;
            std::map<std::string, std::shared_ptr<MeshRectangle>> g_kvCalcMesh;
            std::map<std::string, vector<std::shared_ptr<stRoad>>> g_meshRoads;
            std::map<std::string, map<string, std::shared_ptr<stRoad>>> g_meshRoadIds;
            std::map<std::string, std::shared_ptr<geos::index::quadtree::Quadtree>> g_meshSpatialIndexs;
            std::map<std::string, vector<std::shared_ptr<stRoad>>> g_meshBridgeRoads;
            std::map<std::string, vector<std::shared_ptr<stRoad>>> g_meshMainRoads;
            std::map<std::string, vector<shared_ptr<roadList>>> g_meshRoadGroups;
            std::map<std::string, vector<shared_ptr<roadList>>> g_meshBridgeGroups;
            std::map<std::string, vector<vector<double>>> g_meshBridgeBoundarys;
            std::map<int, vector<string>> fromNodes;
            std::map<int, vector<string>> toNodes;
            std::map<int, vector<string>> fromBridgeNodes;
            std::map<int, vector<string>> toBridgeNodes;

        public:
            std::map<std::string, map<int, vector<std::shared_ptr<stRoad>>>> g_meshProposals;//mesh内候选的任务框
            map<string, map<int, string>> g_meshProposalNames;
            std::map<std::string, map<int, vector<std::shared_ptr<stRoad>>>> g_meshBridgeProposals;//mesh内候选的立交桥任务框
            map<string, map<int, string>> g_meshBridgeProposalNames;
        };
    }
}



#endif //FULL_AUTOMAPPING_CMESH_H
