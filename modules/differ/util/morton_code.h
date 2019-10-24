#ifndef AUTOHDMAP_COMPILE_MORTONCODE_H
#define AUTOHDMAP_COMPILE_MORTONCODE_H

static double           DEF_DEGREE_MAX_COORD        =   180.0f;
static double           DEF_NDS_MAX_COORD           =   2147483648.0f;
static const double     DEF_PRECISION_DOUBLE        =   1e-6;
static unsigned int     DEF_NDS_BASE_X              =   31;
static unsigned int     DEF_NDS_BASE_Y_WITHOUT_SIGN =   29;
static unsigned int     DEF_NDS_BASE_SIGN_Y         =   31;
static unsigned int     DEF_BASE_SIGN_Y_MORTONCODE  =   61;

#include <cmath>
#include <stdint.h>

class MortonCode {

public:
    //MortonCode相关函数
    static int64_t GetMortonCodeFromRAWCoord( const double& x,const double& y );

    static int64_t GetMortonCodeFromNDSCoord(const int& x,const int& y);

    static int GetNDSCoordFromRAW(const double& raw );

    static int GetNDSCoordFromDeg(const double& deg );

    static double ConvertToWgs84Degree(const double& inValue)
    {
        if (std::abs(inValue) > 360.0)
            return inValue / 3600.0;
        else
            return inValue;
    }
};


#endif //AUTOHDMAP_COMPILE_MORTONCODE_H
