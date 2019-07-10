//
// Created by joseph on 2018/8/15.
//

#include "MeshGridObjectExt.hpp"

#include <sstream>

using namespace kd::autohdmap;

void MeshGridObjectExt::setGridID(int nPosX, int nPosY, int nLevel) {
    // 限制在1-5级(世界二级到城市三级)
    if (nLevel < 1) {
        nLevel = 1;
    }
    if (nLevel > 5) {
        nLevel = 5;
    }
    //
    this->gridLevel = nLevel + 1;
    mapRow = (int) ((nPosY / levelLatSize[1]) + 1);
    mapCol = (int) ((nPosX / levelLonSize[1]) + 31);
    //
    mapRow = 'A' + mapRow - 1;
    //
    row = (int) (levelGridCount[nLevel] - (nPosY % levelLatSize[1])
                                          / levelLatSize[nLevel]);
    col = (int) ((nPosX % levelLonSize[1]) / levelLonSize[nLevel] + 1);
    //
    customCol = (int) (((nPosX % levelLonSize[1]) % levelLonSize[nLevel]) / customLonSize[1]);
    customRow = (int) (((nPosY % levelLatSize[1]) % levelLatSize[nLevel]) / customLatSize[1]);
    //
    stringstream gridString;
    gridString << ((char) mapRow);
    gridString << (mapCol);
    gridString << (levelChar[nLevel]);
    //三位
    if (row < 10) {
        gridString << "00" << (row);
    } else {
        if (row < 100) {
            gridString << ("0") << (row);
        } else {
            gridString << (row);
        }
    }
    //三位
    if (col < 10) {
        gridString << ("00") << (col);
    } else {
        if (col < 100) {
            gridString << ("0") << (col);
        } else {
            gridString << (col);
        }
    }

    //
    grid = gridString.str();
}

string MeshGridObjectExt::getGridName(int nCurLevel) {
    if (nCurLevel == -1)
        nCurLevel = gridLevel;
    if (nCurLevel >= gridLevel || (nCurLevel - 1) <= 0) {
        // 如果网格等级比现有的还要详细,则只能返回本级的网格名称
        return grid;
    } else {
        // 百万图幅号肯定保持不变
        stringstream updateGridNameBuffer;
        updateGridNameBuffer << (grid.substr(0, 3));
        // 比例尺代码变化
        updateGridNameBuffer << (levelChar[nCurLevel - 1]);
        // 行列号变化
        int nRow = row / (levelGridCount[gridLevel - 1] / levelGridCount[nCurLevel - 1]) + 1;
        int nCol = col / (levelGridCount[gridLevel - 1] / levelGridCount[nCurLevel - 1]) + 1;
        if (nRow < 10) {
            updateGridNameBuffer << ("00") << (nRow);
        } else {
            if (nRow < 100) {
                updateGridNameBuffer << ("0") << (nRow);
            } else {
                updateGridNameBuffer << (nRow);
            }
        }
        //
        if (nCol < 10) {
            updateGridNameBuffer << ("00") << (nCol);
        } else {
            if (nCol < 100) {
                updateGridNameBuffer << ("0") << (nCol);
            } else {
                updateGridNameBuffer << (nCol);
            }
        }
        return updateGridNameBuffer.str();
    }
}

string MeshGridObjectExt::getCustomGridName() {
    string grid = getGridName(-1);
    stringstream gridString;
    gridString << (grid);
    //
    if (customRow < 10) {
        gridString << ("00") << customRow;
    } else {
        if (customRow < 100) {
            gridString << ("0") << (customRow);
        } else {
            gridString << (customRow);
        }
    }
    //
    if (customCol < 10) {
        gridString << ("00") << (customCol);
    } else {
        if (customCol < 100) {
            gridString << ("0") << (customCol);
        } else {
            gridString << (customCol);
        }
    }
    return gridString.str();
}

void MeshGridObjectExt::setGridName(const string &gridName) {
    this->grid = gridName;
    this->mapRow = gridName[0];
    this->mapCol = stoi(gridName.substr(1, 2));
    //格网级别
    char level = gridName[3];
    //
    int num = sizeof(levelChar) / sizeof(char);
    for (int i = 0; i < num; i++) {
        if (levelChar[i] == level) {
            gridLevel = i + 1;
            break;
        }
    }
    row = stoi(gridName.substr(4, 3));
    col = stoi(gridName.substr(7));
}

void MeshGridObjectExt::getGridArea(MeshRectangle &rect) const {
    rect.xmin = (int) ((mapCol - 31) * levelLonSize[1] + (col - 1) * levelLonSize[gridLevel - 1]);
    rect.ymin = (int) ((mapRow - 'A') * levelLatSize[1] +
                       (levelGridCount[gridLevel - 1] - row) * levelLatSize[gridLevel - 1]);
    rect.xmax = (int) ((mapCol - 31) * levelLonSize[1] + (col) * levelLonSize[gridLevel - 1]);
    rect.ymax = (int) ((mapRow - 'A') * levelLatSize[1] +
                       (levelGridCount[gridLevel - 1] - row + 1) * levelLatSize[gridLevel - 1]);
}

bool MeshGridObjectExt::different(const MeshGridObjectExt &obj, MeshDimension &dim) const {
    if (obj.getGridLevel() != getGridLevel()) {
        // 如果级别不一致,则退出
        return false;
    }
    MeshRectangle rectangle;
    getGridArea(rectangle);
    MeshRectangle rectangle1;
    obj.getGridArea(rectangle1);
    int width = rectangle.getWidth();
    int height = rectangle.getHeight();
    if (rectangle1.xmin >= rectangle.xmin) {
        dim.width = (((rectangle1.xmax + rectangle1.xmin) >> 1) - rectangle.xmin) / width;
    } else {
        dim.width = (((rectangle.xmax + rectangle.xmin) >> 1) - rectangle1.xmin) / width;

    }
    if (rectangle1.ymin >= rectangle.ymin)

        dim.height = (((rectangle1.ymax + rectangle1.ymin) >> 1) - rectangle.ymin) / height;

    else {
        dim.height = (((rectangle.ymax + rectangle.ymin) >> 1) - rectangle1.ymin) / height;

    }
    dim.width = dim.width + 1;
    dim.height = dim.height + 1;
    return true;
}

bool MeshGridObjectExt::offsetGrid(int offsetX, int offsetY) {
    MeshRectangle rectangle;
    getGridArea(rectangle);
    int width = rectangle.getWidth();
    int height = rectangle.getHeight();
    //
    int tempX = rectangle.xmin + offsetX * width;
    int tempY = rectangle.ymin + offsetY * height;
    // 如果大于180度 则转为相应的负值
    if (tempX > maxGridRect.xmax)
        tempX -= maxGridRect.getWidth();
    // 如果小于-180度 则转为相应的正值
    if (tempX < maxGridRect.xmin)
        tempX += maxGridRect.getWidth();
    // 如果大于90度跳回
    if (tempY >= (maxGridRect.ymax - LONGLAT_RATIO))
        tempY = maxGridRect.ymax - LONGLAT_RATIO;
    // 如果小于－90度跳回
    if (tempY <= (maxGridRect.ymin + LONGLAT_RATIO))
        tempY = maxGridRect.ymin + LONGLAT_RATIO;

    //tempY = tempY + (height >> 1);
    //tempX = tempX + (width >> 1);
    setGridID(tempX, tempY, getGridLevel() - 1);
    return true;
}