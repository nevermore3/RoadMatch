#ifndef ROAD_MATCH_CORE_ATTRIBUTE_CONVERT_H
#define ROAD_MATCH_CORE_ATTRIBUTE_CONVERT_H

#include <stdint.h>

class AttributeConv {
public:
    static int32_t ConvertRoadClass(int32_t roadclass);

    static int32_t ConvertFC(int32_t fc);

    static int32_t ConvertFormWay(int32_t formway);

    static int32_t ConvertTollFlag(int32_t toll);

    static int32_t ConvertOwnerShip(int32_t owner);

    static int32_t ConvertStatus(int32_t status);

};

#endif //ROAD_MATCH_CORE_ATTRIBUTE_CONVERT_H
