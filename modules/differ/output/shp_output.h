#ifndef SDMAP_DIFFER_FUSION_SHP_OUTPUT_H
#define SDMAP_DIFFER_FUSION_SHP_OUTPUT_H

#include "data_manager/data_types.h"

class ShpOutPut {
public:
    static void OutPutLinkList(const string &file,
                               const list<shared_ptr<KDRoad>>& roads);

};


#endif //SDMAP_DIFFER_FUSION_SHP_OUTPUT_H
