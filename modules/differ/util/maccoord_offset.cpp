#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "util/maccoord_offset.h"
//本地全局变量
int nDefTime_MABC;
char nDefWeek_MABC;
int nDayBase_MABC;
int nValidTime_MABC=1;
//
int casm_t1_MABC;
int casm_t2_MABC;
double casm_rr_MABC;
double casm_x1_MABC;
double casm_x2_MABC;
double casm_y1_MABC;
double casm_y2_MABC;
double casm_f_MABC;

//前置声明
unsigned int  mac_IniCasm_MABC(int a1, unsigned int a2, unsigned int a3);
long double  mac_random_yj_MABC();
long double  mac_yj_sin2_MABC(double a1);
int  mac_IsValidParams_MABC(int *a1, int *a2);
int  mac_GetDays_MABC(signed int a1, int a2, int a3);
int  mac_GetTimeParams_MABC(int *a1, int *a2);
int  mac_InitTimeParams_MABC();

long double  mac_Transform_yj5_MABC(double a1, double a2);
long double  mac_Transform_jyj5_MABC(double a1, double a2);
long double  mac_Transform_yjy5_MABC(double a1, double a2);
long double  mac_Transform_jy5_MABC(double a1, double a2);
//
int  mac_wgtochina_lb_MABC(int a1, unsigned int a2, unsigned int a3, int a4, int a5, int a60, int * a6, int * a7);
int  mac_WG2China_MABC(unsigned int a1, unsigned int a2, int * a3, int * a4);
int  mac_WG2China_MABC_d(double a1, double a2, double * a3, double * a4);
void mac_ReadConfig_MABC();
int  mac_InitWG2China_MABC();

//实现
unsigned int  mac_IniCasm_MABC(int a1, unsigned int a2, unsigned int a3) {
    unsigned int result; // eax@3
    long double v4; // fst7@3
    long double v5; // fst6@3
    
    casm_t1_MABC = a1;
    casm_t2_MABC = a1;
    *(double *) &casm_rr_MABC = (long double) (unsigned int) a1
    - 0.357 * (long double) (signed int) ((long double) (unsigned int) a1 / 0.357);
    if (!a1)
        casm_rr_MABC = 4599075939470750515LL;
    v4 = (long double) a2;
    result = a3;
    *(double *) &casm_x1_MABC = v4;
    v5 = (long double) a3;
    *(double *) &casm_y1_MABC = v5;
    *(double *) &casm_x2_MABC = v4;
    *(double *) &casm_y2_MABC = v5;
    casm_f_MABC = 4613937818241073152LL;
    return result;
}
long double  mac_random_yj_MABC() {
    long double result; // fst7@1
    long double v1; // fst7@1
    
    v1 = *(double *) & casm_rr_MABC * (long double) 314159269 + (long double) 453806245;
    result = 0.5 * (v1 - (long double) (2 * (signed int) (v1 * 0.5)));
    *(double *) &casm_rr_MABC = result;
    return result;
}
long double  mac_yj_sin2_MABC(double a1) {
    signed int v1; // edx@1
    long double v2; // fst7@1
    long double v3; // fst6@3
    long double v4; // fst7@4
    long double v5; // fst6@7
    long double result; // fst7@7
    long double v7; // fst5@7
    long double v8; // fst7@7
    
    v2 = a1;
    v1 = 0;
    if (a1 < 0.0) {
        v2 = -v2;
        v1 = 1;
    }
    v3 = v2 - 6.28318530717959 * (long double) (signed int) (v2 / 6.28318530717959);
    if (v3 <= 3.141592653589793) {
        v4 = v3;
    } else {
        v4 = v3 - 3.141592653589793;
        if (v1 == 1) {
            v1 = 0;
        } else {
            if (!v1)
                v1 = 1;
        }
    }
    v5 = v4;
    v7 = v4 * v4;
    v8 = v4 * v4 * v4;
    result = v8 * v7 * v7 * v7 * 0.00000275573192239859
    + v8 * v7 * 0.00833333333333333
    + v5
    - v8 * 0.166666666666667
    - v8 * v7 * v7 * 0.000198412698412698
    - v7 * v8 * v7 * v7 * v7 * 0.0000000250521083854417;
    if (v1 == 1)
        result = -result;
    return result;
    
}
int  mac_IsValidParams_MABC(int *a1, int *a2) {
    signed int result; // eax@2
    
    if (*(int *) & nDefWeek_MABC) {
        *(int *) a1 = *(int *) & nDefWeek_MABC;
        *(int *) a2 = nDefTime_MABC;
        result = 1;
    } else {
        result = 0;
    }
    return result;
}
int  mac_GetDays_MABC(signed int a1, int a2, int a3) {
    //
    static char nDaysOfMonthl [] = {0, 0, 0, 0, 0, 0, 0, 0, 0x1f, 0, 0, 0, 0x3c,
        0, 0, 0, 0x5b, 0, 0, 0, 0x79, 0, 0, 0, static_cast<char>(0x98), 0, 0, 0, static_cast<char>(0xb6), 0, 0, 0, static_cast<char>(0xd5),
        0, 0, 0, static_cast<char>(0xf4), 0, 0, 0, 0x12, 1, 0, 0, 0x31, 1, 0, 0, 0x4f, 1, 0, 0};
    static char nDaysOfMonthc [] = {0, 0, 0, 0, 0, 0, 0, 0, 0x1f, 0, 0, 0, 0x3b,
        0, 0, 0, 0x5a, 0, 0, 0, 0x78, 0, 0, 0, static_cast<char>(0x97), 0, 0, 0, static_cast<char>(0xb5), 0, 0, 0, static_cast<char>(0xd4),
        0, 0, 0, static_cast<char>(0xf3), 0, 0, 0, 0x11, 1, 0, 0, 0x30, 1, 0, 0, 0x4e, 1, 0, 0};
    signed int v3; // esi@2
    int v4; // ecx@6
    
    if (a1 & 3) {
        v3 = 0;
    } else {
        v3 = 0;
        if (a1 % 100 || !(a1 % 400))
            v3 = 1;
    }
    if (v3)
        v4 = *(int *) & nDaysOfMonthl[4 * a2] + a3;
    else
        v4 = *(int *) & nDaysOfMonthc[4 * a2] + a3;
    return (a1 - 1) / 400 + (a1 - 1) / 4 + 365 * (a1 - 1) + v4 - 1 - (a1 - 1) / 100;
}
/**
 * 获得时间参数
 *
 *
 */
int  mac_GetTimeParams_MABC(int *a1, int *a2) {
    int result; // eax@1
    struct tm *v3; // eax@3
    signed int v4; // ebx@3
    time_t timer; // [sp+1Ch] [bp-14h]@3
    
    result = mac_IsValidParams_MABC(a1, a2);
    if (!result) {
        time(&timer);
        //unsafe API
        v3 = localtime(&timer);
        v4 = mac_GetDays_MABC(v3->tm_year + 1900, v3->tm_mon + 1, v3->tm_mday) - nDayBase_MABC;
        *a1 = v4 / 7;
        result = timer / 86400;
        *a2 = 1000 * (timer % 86400 + 86400 * v4 % 7);
    }
    return result;
}
int mac_IsValidTime(int year,int month,int day){
    time_t timer;
    struct tm *v3;
    time(&timer);
    //unsafe API
    v3 = localtime(&timer);
    if((year-1900)>=v3->tm_year&&(month-1)>=v3->tm_mon&&day>=v3->tm_mday){
        nValidTime_MABC=1;
        return 1;
    }else{
        nValidTime_MABC=0;
        return 0;
    }
    
}
long double mac_Transform_yj5_MABC(double a1, double a2) {
    long double v2=0;
    long double v3; // fst7@1
    short v4=0; // fps@1
    char v5; // c0@1
    char v6; // c2@1
    char v7; // c3@1
    long double v8; // fst7@2
    long double v9; // fst6@3
    short v10=0; // fps@3
    char v11; // c0@3
    char v12; // c2@3
    char v13; // c3@3
    long double v14; // fst7@4
    double v16; // ST30_8@5
    double v17; // ST10_8@5
    double v18; // ST30_8@5
    double v19; // ST10_8@5
    double v20; // ST30_8@5
    double v21; // ST10_8@5
    
    v3 = a1 * a1;
    //
    v2 = sqrt(v3);
    //UNDEF(v4);
    v5 = v2 < v2;
    v6 = 0;
    v7 = v2 == v2;
    if ((((v4 >> 8)&0xFF)& 0x45) == 64)
        v8 = v2;
    else
        v8 = sqrt(v3);
    v9 = sqrt(v8);
    //UNDEF(v10);
    v11 = v9 < v9;
    v12 = 0;
    v13 = v9 == v9;
    if ((((v10 >> 8)&0xFF) & 0x45) == 64)
        v14 = v9;
    else
        v14 = sqrt(v8);
    v16 = (double) (a2 * a1 * 0.1 + a1 * a1 * 0.1 + a2 + a2 + (double) (a1 + 300.0)) + (double) (v14 * 0.1);
    v17 = mac_yj_sin2_MABC(a1 * 18.84955592153876) * 20.0;
    v18 = (mac_yj_sin2_MABC(a1 * 6.283185307179588) * 20.0 + v17) * 0.6667 + v16;
    v19 = mac_yj_sin2_MABC(a1 * 3.141592653589794) * 20.0;
    v20 = (mac_yj_sin2_MABC(a1 * 1.047197551196598) * 40.0 + v19) * 0.6667 + v18;
    v21 = mac_yj_sin2_MABC(a1 * 0.2617993877991495) * 150.0;
    return (mac_yj_sin2_MABC(a1 * 0.1047197551196598) * 300.0 + v21) * 0.6667 + v20;
    
}

void mac_ReadConfig_MABC() {
    nDefTime_MABC = 0;
    nDefWeek_MABC = 0;
    nDayBase_MABC = 0;
    //
    
}
long double  mac_Transform_jyj5_MABC(double a1, double a2) {
    long double v2; // fst6@1
    long double v3; // fst7@1
    double v4; // ST20_8@1
    double v5; // ST30_8@1
    short v6=0; // fps@1
    char v7; // c0@1
    char v8; // c2@1
    char v9; // c3@1
    double v11; // ST20_8@3
    
    v4 = 0.0174532925199433 * a1;
    v5 = mac_yj_sin2_MABC(v4) * (double) 0.00669342;
    v3 = 1.0 - mac_yj_sin2_MABC(v4) * v5;
    v2 = sqrt(v3);
    //UNDEF(v6);
    v7 = v2 < v2;
    v8 = 0;
    v9 = v2 == v2;
    if ((((v6 >> 8)&0xff) & 0x45) != 64) {
        v11 = v3;
        v2 = sqrt(v3);
        v3 = v11;
    }
    return a2 * 180.0 / (6335552.7273521 / (v3 * v2) * 3.1415926);
}

int  mac_InitTimeParams_MABC() {
    int result; // eax@1
    result = mac_GetDays_MABC(1980, 1, 6);
    nDayBase_MABC = result;
    return result;
}

long double mac_Transform_yjy5_MABC(double a1, double a2) {
    long double v2; // fst5@1
    long double v3; // fst6@1
    long double v4; // fst7@1
    short v5=0; // fps@1
    char v6; // c0@1
    char v7; // c2@1
    char v8; // c3@1
    long double v9; // fst6@2
    long double v10; // fst5@3
    short v11=0; // fps@3
    char v12; // c0@3
    char v13; // c2@3
    char v14; // c3@3
    long double v15; // fst6@4
    double v17; // ST38_8@5
    double v18; // ST20_8@5
    double v19; // ST38_8@5
    double v20; // ST20_8@5
    double v21; // ST38_8@5
    double v22; // ST20_8@5
    double v23; // ST10_8@6
    double v24; // ST10_8@7
    
    v4 = a1 + a1 + -100.0 + a2 * 3.0 + a2 * 0.2 * a2 + a1 * 0.1 * a2;
    v3 = a1 * a1;
    v2 = sqrt(v3);
    //UNDEF(v5);
    v6 = v2 < v2;
    v7 = 0;
    v8 = v2 == v2;
    if ((((v5 >> 8)&0xff) & 0x45) == 64) {
        v9 = v2;
    } else {
        v24 = v4;
        v9 = sqrt(v3);
        v4 = v24;
    }
    v10 = sqrt(v9);
    //UNDEF(v11);
    v12 = v10 < v10;
    v13 = 0;
    v14 = v10 == v10;
    if ((((v11 >> 8)&0xff) & 0x45) == 64) {
        v15 = v10;
    } else {
        v23 = v4;
        v15 = sqrt(v9);
        v4 = v23;
    }
    v17 = v4 + v15 * 0.2;
    v18 = mac_yj_sin2_MABC(a1 * 18.84955592153876) * 20.0;
    v19 = (mac_yj_sin2_MABC(a1 * 6.283185307179588) * 20.0 + v18) * 0.6667 + v17;
    v20 = mac_yj_sin2_MABC(a2 * 3.141592653589794) * 20.0;
    v21 = (mac_yj_sin2_MABC(a2 * 1.047197551196598) * 40.0 + v20) * 0.6667 + v19;
    v22 = mac_yj_sin2_MABC(a2 * 0.2617993877991495) * 160.0;
    return (mac_yj_sin2_MABC(a2 * 0.1047197551196598) * 320.0 + v22) * 0.6667 + v21;
}

long double  mac_Transform_jy5_MABC(double a1, double a2) {
    long double v2; // fst6@1
    long double v3; // fst7@1
    long double v4; // fst7@1
    
    
    v4 = mac_yj_sin2_MABC(a1 * 0.0174532925199433);
    v3 = 1.0 - mac_yj_sin2_MABC(a1 * 0.0174532925199433) * (double) (v4 * (double) 0.00669342);
    v2 = sqrt(v3);
    //
    return (double) (a2 * 180.0) / (cos(a1 * 0.0174532925199433) * (double) (6378245.0 / v2) * 3.1415926);
}

/**
 * 关键函数 计算真正的偏移值
 * @param a1 是否是初始化标识
 * @param a2 经度
 * @param a3 纬度
 * @param a4 目前这个值是50
 * @param a5
 */
int  mac_wgtochina_lb_MABC(int wg_flag, unsigned int wg_lng, unsigned int wg_lat, int wg_heit, int wg_week, int wg_time, int * china_lng, int * china_lat) {
    int result; // eax@6
    double v8; // ST50_8@11
    double v9; // ST38_8@11
    double v10; // ST30_8@11
    long double v11; // fst7@11
    double v12; // ST48_8@11
    double v13; // ST40_8@11
    double v14; // ST38_8@11
    double v15; // ST38_8@11
    double v16;
    double v17;
    double v18=wg_lat/3686400.0;
    double v19=wg_lng/3686400.0;
    //
    //检查坐标范围
    if (wg_heit <= 5000&&v19 >=72.004&& v19 <= 137.8347&& v18 >= 0.8293&& v18 <= 55.8271)
    {
        if (wg_flag) {
            casm_t2_MABC = wg_time;
            v8 = v18 - 35.0;
            //
            v9 = mac_Transform_yj5_MABC(v19 - 105.0, v8);
            v10 = mac_Transform_yjy5_MABC(v19 - 105.0, v8);
            v11 = 0.001 *wg_heit;
            v12 = v11;
            //
            v13 = wg_time* 0.0174532925199433;
            v14 = mac_yj_sin2_MABC(v13) + (v9 + v11);
            v15 = /*random_yj()*/ + v14;
            //
            v16 = mac_yj_sin2_MABC(v13) + v10 + v12;
            v17 = /*random_yj()*/ + v16;
            
            //反投影
            *china_lng= (mac_Transform_jy5_MABC(v18, v15) + v19) * 3686400.0;
            *china_lat= (mac_Transform_jyj5_MABC(v18, v17) + v18) * 3686400.0;
        }
        else{
            //
            mac_IniCasm_MABC(wg_time, wg_lng, wg_lat);
            *china_lng = wg_lng;
            *china_lat= wg_lat;
        }
        result = 0;
    }else {
        //不在中国范围内？
        (*china_lng) = 0;
        (*china_lat) = 0;
        result = -27137;
    }
    return result;
}

/**
 * 进行坐标偏移
 * @param a1 经度
 * @param a2 纬度
 * @param a3 输出经度
 * @param a4 输出纬度
 */
int  mac_WG2China_MABC(unsigned int a1, unsigned int a2, int * a3, int * a4) {
    int result;
    int v6=0; //
    int v7=0; //
    //GetTimeParams(&v6, &v7);
    //
    result = mac_wgtochina_lb_MABC(1, a1, a2, 50, v6, v7, a3, a4);
    if (result!=0) {
        //
        *a3 = 2147483647;
        *a4 = 2147483647;
    }
    return result;
}

int  mac_WG2China_MABC_d(double a1, double a2, double * a3, double * a4) {
    int result;
    int v5=0; 
    int v6=0; 
    
    result=mac_WG2China_MABC((unsigned int) (a1 * 3686400.0), (unsigned int) (a2 * 3686400.0), &v5, &v6);
    if(result==0){
        *a3 = v5 / (double) 3686400.0;
        //
        *a4 = v6 / (double) 3686400.0;
    }else{
        *a3=a1;
        *a4=a2;
        result=-2;//不在范围内
    }
    return result;
}
int  mac_InitWG2China_MABC() {
    
    int v2; //
    int v3; //
    int v4; //
    int v5; //
    
    mac_GetTimeParams_MABC(&v4, &v5);
    //
    //0表示为初始化
    return mac_wgtochina_lb_MABC(0, 0x19938000u, 0x8C46000u, 50, v4, v5, &v2, & v3);
}

int  mac_coord_init(){
    mac_ReadConfig_MABC();
    mac_InitWG2China_MABC();
    return 1;//IsValidTime(2012,9,30);
}
/**
 *
 */
int  mac_coord_offset(double x, double y, double * cx, double * cy){
    if(nValidTime_MABC==1){
        return mac_WG2China_MABC_d(x, y, cx, cy);
    }else{
        (*cx)=x;
        (*cy)=y;
        return -1;
    }
}
int  mac_coord_deoffset(double x, double y, double * cx, double * cy){
    if(nValidTime_MABC==1){
        double x1,y1;
        double dx,dy;
        int status=mac_WG2China_MABC_d(x, y, &x1, &y1);
        dx=x1-x;
        dy=y1-y;
        (*cx)=x-dx;
        (*cy)=y-dy;
        return status;
    }else{
        (*cx)=x;
        (*cy)=y;
        return -1;
    }
}
