//
// Created by cairunbo on 18-10-31.
//

#include "util/morton_code.h"

int64_t MortonCode::GetMortonCodeFromRAWCoord( const double& x,const double& y )
{
    return GetMortonCodeFromNDSCoord( GetNDSCoordFromRAW(x), GetNDSCoordFromRAW(y) );
}

int64_t MortonCode::GetMortonCodeFromNDSCoord( const int& x,const int& y )
{
    int64_t mortonCode = 0;
    int64_t val = 1;

    // X-Coordinate(0-31bit)
    for ( unsigned int i = 0; i <= DEF_NDS_BASE_X; i++ )
    {
        if ( x & (1<<i) )
        {
            mortonCode += (val<<(i*2));
        }
    }

    // Y-Coordinate(0-29bit)
    for ( unsigned int i = 0; i <= DEF_NDS_BASE_Y_WITHOUT_SIGN; i++ )
    {
        if ( y & (1<<i) )
        {
            mortonCode += (val<<((i*2)+1));
        }
    }

    // Y-Coordinate(31bit)
    // * 30bit is not use.
    // * 30bit explains between 90Deg.~180Deg. / 31 bit explains Plus or Minus.
    if ( y & (1<<DEF_NDS_BASE_SIGN_Y) )
    {
        mortonCode += (val<<DEF_BASE_SIGN_Y_MORTONCODE);
    }
    return mortonCode;
}

int MortonCode::GetNDSCoordFromRAW(const double& raw )
{
    return	GetNDSCoordFromDeg(ConvertToWgs84Degree(raw));
}

int MortonCode::GetNDSCoordFromDeg(const double& deg )
{
    if (std::abs(deg - 180.0) <= DEF_PRECISION_DOUBLE)
    {
        return (DEF_NDS_MAX_COORD - 1);
    }
    return static_cast<int>( (deg * DEF_NDS_MAX_COORD / DEF_DEGREE_MAX_COORD) + 0.5f );
}