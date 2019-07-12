//
// Created by liujian on 19-7-12.
//

#include "shp_output.h"

#include <shp/ShpData.hpp>

void ShpOutPut::OutPutLinkList(const string &file,
                               const list<shared_ptr<KDRoad>>& roads) {
    string dbf_file = file + ".dbf";
    string shp_file = file + ".shp";

    SHPHandle ptrRoadShp_ = SHPCreate(shp_file.c_str(), SHPT_ARCZ);;
    DBFHandle ptrRoadDbf_ = DBFCreate(dbf_file.c_str());

    DBFAddField(ptrRoadDbf_, "ID", FTLong, 16, 0);
    DBFAddField(ptrRoadDbf_, "MESHID", FTString, 16, 0);
    DBFAddField(ptrRoadDbf_, "DIRECTION", FTInteger, 10, 0);
    DBFAddField(ptrRoadDbf_, "SNODE_ID", FTLong, 16, 0);
    DBFAddField(ptrRoadDbf_, "ENODE_ID", FTLong, 16, 0);

    int nCount = 0;
        for(auto road: roads) {
            //write geo object
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

    if (ptrRoadShp_ != nullptr) {
        SHPClose(ptrRoadShp_);
    }
    if (ptrRoadDbf_ != nullptr) {
        DBFClose(ptrRoadDbf_);
    }
}