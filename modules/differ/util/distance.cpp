//
// Created by Liu Jian on 2018/10/10.
//

#include "distance.h"

#include <limits.h>

//const double PI = 3.141592653589793;
//const double deg2rad = PI / 180.0;
//const double radius = 6371004.0;
//#define EARTH_2R			12733467.287768236971495736012709F
//#define FACTOR_ARC_RAD		57295779.513082320876798154814105F
const double PI = 3.141592653589793;
const double deg2rad = PI / 180.0;
#define EARTH_2R			1273346728.7768236971495736012709F
#define FACTOR_ARC_RAD		5729577951.3082320876798154814105F

double GisToolSetGetLonLatDist(long coor1x, long coor1y, long coor2x, long coor2y)
{
/*	double dlon = FLTARC2RAD(coor2x - coor1x);
	double dlat = FLTARC2RAD(coor2y - coor1y);
	double sindlat2 = PMSin(FDIV(dlat,2.0));
	double sindlon2 = PMSin(FDIV(dlon,2.0));
	double a = FADD(FMUL(sindlat2, sindlat2),
		FMUL(FMUL(PMCos(FLTARC2RAD(coor1y)), PMCos(FLTARC2RAD(coor2y))), FMUL(sindlon2,sindlon2)));
	return FMUL(EARTH_2R, PMArcSin(FSQRT(a)));
*/
	if (coor1x == coor2x && coor1y == coor2y) {
		return 0;
	}

	double dlon = (coor2x - coor1x) / FACTOR_ARC_RAD;
	double dlat = (coor2y - coor1y) / FACTOR_ARC_RAD;
	double sindlat2 = sin(dlat / 2.0);
	double sindlon2 = sin(dlon / 2.0);
	double a = sindlat2 * sindlat2 + cos(coor1y / FACTOR_ARC_RAD) * cos(coor2y / FACTOR_ARC_RAD) * sindlon2 * sindlon2;
	return EARTH_2R * asin(sqrt(a));
}

double Distance::distance(const shared_ptr<KDCoord> pt1, const shared_ptr<KDCoord> pt2) {
//	double x1 = pt1.get_double_lon();
//	double y1 = pt1.get_double_lat();
//	double x2 = pt2.get_double_lon();
//	double y2 = pt2.get_double_lat();

	return GisToolSetGetLonLatDist(pt1->lng_ * 100000000.0, pt1->lat_ * 100000000.0,
                                   pt2->lng_ * 100000000.0, pt2->lat_ * 100000000.0);

//	return radius * acos(sin(y1 * deg2rad) * sin(y2 * deg2rad)
//		+ cos(y1 * deg2rad) * cos(y2 * deg2rad) * cos(abs(x1 * deg2rad - x2 * deg2rad)));
}

long latlonRound(double value){
	if (value >= 0) {
		return (long)(value + 0.5);
	}else{
		return (long)(value - 0.5);
	}
}

double Distance::distance(const shared_ptr<KDCoord> pt,
						  const vector<shared_ptr<KDCoord>>& line,
                          shared_ptr<KDCoord> pFoot,
						  int32_t* pSeg) {
	double dMinDistance = INT_MAX;
	int ptNum = line.size();

	for (int i = 0; i < ptNum - 1; i++) {
		shared_ptr<KDCoord> a = line[i];
		shared_ptr<KDCoord> b = line[i + 1];

		shared_ptr<KDCoord> c = make_shared<KDCoord>();
		shared_ptr<KDCoord> ab = make_shared<KDCoord>();
        ab->lat_ = b->lat_  - a->lat_ ;
        ab->lng_ = b->lng_  - a->lng_ ;
		shared_ptr<KDCoord> ac = make_shared<KDCoord>();
		ac->lat_ = pt->lat_ - a->lat_;
        ac->lng_ = pt->lng_ - a->lng_ ;

		double cosLat = cos(pt->lat_ * deg2rad);
        double f = ab->lng_ * 100000000.0 * ac->lng_ * 100000000.0 * cosLat * cosLat +
                    ab->lat_ * 100000000.0 * ac->lat_ * 100000000.0;
        double d = ab->lng_ * 100000000.0 * ab->lng_ * 100000000 * cosLat * cosLat +
                    ab->lat_ * 100000000.0 * ab->lat_ * 100000000.0;

		double dDis = INT_MAX;
		if (abs(d) < 0.0000001) {
			// dDis=0;
			dDis = distance(a, pt);
		} else if (f < 0) {
			// Distance(a, c);
			dDis = distance(a, pt);
		} else if (f > d) {
			// Distance(b, c)
			dDis = distance(b, pt);
		} else {
			double newf = f / d;
			//use our own defined function to adjust the accuracy
			c->lng_ = a->lng_ + newf * ab->lng_;
			c->lat_ = a->lat_ + newf * ab->lat_ ;
			dDis = distance(c, pt);
		}

		if (dDis < dMinDistance) {
			dMinDistance = dDis;
			if (pSeg) {
				*pSeg = i;
			}

			if (pFoot) {
				if (abs(d) < 0.0000001) {
					pFoot->lng_ = a->lng_;
					pFoot->lat_ = a->lat_;
				} else if (f < 0) {
                    pFoot->lng_ = a->lng_;
                    pFoot->lat_ = a->lat_;
				} else if (f > d) {
                    pFoot->lng_ = b->lng_;
                    pFoot->lat_ = b->lat_;
					// *pFoot = i + 1;
				} else {
                    pFoot->lng_ = c->lng_;
                    pFoot->lat_ = c->lat_;
				}
			}
		}
	}

	return dMinDistance;
}
