//
// Created by Liu Jian on 2018/10/9.
//

#ifndef NDS_VIEWER_DISTANCE_H
#define NDS_VIEWER_DISTANCE_H

#include "data_manager/data_types.h"

#include <list>

class Distance {
    public:
        static double distance(const shared_ptr<KDCoord> pt1, const shared_ptr<KDCoord> pt2);

        static double distance(const shared_ptr<KDCoord> pt,
                               const vector<shared_ptr<KDCoord>>& line,
                                shared_ptr<KDCoord> pFoot,
                               int32_t* pSeg);
};

#endif //NDS_VIEWER_DISTANCE_H
