//
// Created by liujian on 19-7-5.
//

#ifndef TD_DATA_DIFFER_DATA_COMPARE_H
#define TD_DATA_DIFFER_DATA_COMPARE_H

#include "icompare.h"

class DataCompare {
public:
    DataCompare() = default;
    ~DataCompare() = default;

    bool Init() {return true;};

    bool CompareData() {return true;};

public:
    vector<ICompare*> comparer_list_;
};

class RoadCompare : ICompare {
public:
    RoadCompare() = default;
    ~RoadCompare() = default;

public:
    bool Compare() {};
};

class NodeCompare : ICompare {
public:
    NodeCompare() = default;
    ~NodeCompare() = default;

public:
    bool Compare() {} override;
};

class TopoCompare : ICompare {
public:
    TopoCompare() = default;
    ~TopoCompare() = default;

public:
    bool Compare() {} override;
};




#endif //TD_DATA_DIFFER_DATA_COMPARE_H
