#ifndef _MAC_COORD_OFFSET_HPP
#define _MAC_COORD_OFFSET_HPP
#ifdef __cplusplus
extern "C"{
#endif /* __cplusplus */

int mac_coord_init();

int mac_coord_offset(double x, double y, double * x1, double * y1);
int mac_coord_deoffset(double x, double y, double * cx, double * cy);

#ifdef __cplusplus
};
#endif

#endif //_MAC_COORD_OFFSET_HPP
