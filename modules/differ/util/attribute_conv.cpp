#include "attribute_conv.h"

int32_t AttributeConv::ConvertRoadClass(int32_t roadclass) {
    int32_t res_roadclass;
    switch(roadclass) {
        case 41000: //高速
            res_roadclass = 10000;
            break;
        case 42000: //国道
            res_roadclass = 11000;
            break;  //城市快速路
        case 43000:
            res_roadclass = 12000;
            break;
        case 44000: //城市主干路
            res_roadclass = 13000;
            break;
        case 45000: //城市次干路
            res_roadclass = 14000;
            break;
        case 47000: //普通道路
            res_roadclass = 15000;
            break;
        case 51000: //省道
            res_roadclass = 16000;
            break;  //县道
        case 52000:
            res_roadclass = 17000;
            break;
        case 53000: //乡道
            res_roadclass = 18000;
            break;
        case 54000: //县乡村内部道路
        case 49:    //小路
            res_roadclass = 19000;
            break;
        default:
            res_roadclass = 10000;
            break;
    }

    return res_roadclass;
}

int32_t AttributeConv::ConvertFC(int32_t fc) {
    return fc + 1; //参考数据01234，交换规格数据123456
}

int32_t AttributeConv::ConvertFormWay(int32_t formway) {
    int32_t  res_formway;
    switch(formway) {
        case 12: //右转车道B
            res_formway = 11;
            break;
        case 13: //左转车道A
        case 14: //左转车道B
            res_formway = 12;
            break;
        case 59: //门前道路（kxs暂无此类型，暂时映射至普通道路）
        case 15: // 普通道路
            res_formway = 14;
            break;
        case 16: //左右转车道
            res_formway = 13;
            break;
        case 17: //非机动车借道
            res_formway = 15;
            break;
        case 53: //服务区&JCT
            res_formway = 17;
            break;
        case 56: //服务区&引路
            res_formway = 16;
            break;
        case 58: //服务区&引路&JCT
            res_formway = 18;
            break;
        default: //其它类型交换规格与参考数据相同。
            res_formway = formway;
            break;
    }

    return res_formway;
}

int32_t AttributeConv::ConvertTollFlag(int32_t toll) {
    if (toll == 2) //非收费道路
        return 0;
    else
        return 1;
}

int32_t AttributeConv::ConvertOwnerShip(int32_t ownership) {
    int32_t res_ownership;
    switch(ownership) {
        case 0: //公共道路
            res_ownership = 1;
            break;
        case 1: //内部道路
            res_ownership = 2;
            break;
        case 2: //私有道路
            res_ownership = 3;
            break;
        case 3: //地下停车场道路
        case 4: //立体停车场道路
            res_ownership = 4;
            break;
        default:
            res_ownership = 1;
            break;
    }
    return res_ownership;
}

int32_t AttributeConv::ConvertStatus(int32_t status)  {
    return status + 1; //参考数据012，交换规格数据123
}