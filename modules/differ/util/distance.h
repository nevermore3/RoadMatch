//
// Created by Liu Jian on 2018/10/9.
//

#ifndef NDS_VIEWER_DISTANCE_H
#define NDS_VIEWER_DISTANCE_H

#include "data_manager/data_types.h"

#include <list>

class Distance {
    public:
        static double SimpleDistance(const shared_ptr<KDCoord>& pt1, const shared_ptr<KDCoord>& pt2);
        static double SimpleDistance(double x1,double y1,double x2,double y2);
        static double distance(double lng1,double lat1,double lng2,double lat2);
        static double distance(const shared_ptr<KDCoord> pt1, const shared_ptr<KDCoord> pt2);


        static double distance(const shared_ptr<KDCoord> pt,
                               const vector<shared_ptr<KDCoord>>& line,
                               shared_ptr<KDCoord> pFoot,
                               int32_t* pSeg);

        static double distance(const shared_ptr<KDCoord> pt,
                               const vector<shared_ptr<KDCoord>>& line,
                               shared_ptr<KDCoord> pFoot,
                               int32_t* pSeg, int8_t* locate, int start = 0,
                               int end = -1);
        // default: locate=nullptr , start = 0, end = -1;

        static double GetDegreeDistance(double meter_dis,double lon,double lat);

        /**
        * 获取长度，不进行单位转化
        * @param line
        * @return
        */
        static double GetSimpleLength(const vector<shared_ptr<KDCoord>>& line);

};

#endif //NDS_VIEWER_DISTANCE_H
