#ifndef TD_DATA_DIFFER_ICOMPARE_H
#define TD_DATA_DIFFER_ICOMPARE_H

class ICompare {
public:
    virtual ~ICompare() = default;

    virtual bool Compare() = 0;
};

#endif //TD_DATA_DIFFER_ICOMPARE_H
