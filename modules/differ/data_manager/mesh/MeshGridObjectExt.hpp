#ifndef AUTOHDMAP_GEO_MESHGRIDOBJECTEXT_HPP
#define AUTOHDMAP_GEO_MESHGRIDOBJECTEXT_HPP

#include <string>
#include <cmath>

using namespace std;

namespace kd {
    namespace autohdmap {

        struct MeshRectangle {
            int xmin;
            int ymin;
            int xmax;
            int ymax;
        public:
            MeshRectangle() {}

            MeshRectangle(int _xmin, int _ymin, int _xmax, int _ymax) : xmin(_xmin), ymin(_ymin), xmax(_xmax),
                                                                        ymax(_ymax) {

            }

        public:
            int getHeight() const {
                return ymax - ymin;
            }

            int getWidth() const {
                return xmax - xmin;
            }
        };

        struct MeshDimension {
            int width;
            int height;
        public:
            MeshDimension() {

            }

            MeshDimension(const MeshDimension &pObj) {
                width = pObj.width;
                height = pObj.height;
            }
        };

        /**
         * 标准地图分幅
         */
        class MeshGridObjectExt {
        public:
            MeshGridObjectExt() : gridLevel(1) {

            }

            ~MeshGridObjectExt() = default;

        public:
            void setGridID(int nPosX, int nPosY, int nLevel);

            string getGridName(int nCurLevel = -1);

            string getCustomGridName();

            void setGridName(const string &gridName);

            void getGridArea(MeshRectangle &rect) const;

            bool different(const MeshGridObjectExt &obj, MeshDimension &dim) const;


            bool offsetGrid(int offsetX, int offsetY);

        public:
            int getGridLevel() const {
                return this->gridLevel;
            }

            int getOffsetBit() const {
                return (int) (gridOffsetBit[gridLevel - 1]);
            }

            int getLevelY(double y) const {
                y = std::abs(y);
                if (y <= DBL_EPSILON_OWN)
                    y = 0.0;
                return (((int) (yRatio * y + 0.5)) >> getOffsetBit());
            }

            int getLevelX(double x) const {
                x = std::abs(x);
                if (x <= DBL_EPSILON_OWN)
                    x = 0.0;
                return (((int) (xRatio * x + 0.5)) >> getOffsetBit());
            }

        private:
            int gridLevel;
            // 百万图幅的行号和列号
            int mapRow;
            int mapCol;
            // 百万图幅内部的行号和列号
            int row;
            int col;
            // 百万图幅内部用户自定义网格大小编号XXX YYY
            int customRow;
            int customCol;
            //
            string grid;
        private:
            const int LONGLAT_RATIO = 3600000;
            const MeshRectangle maxGridRect = {-180 * LONGLAT_RATIO, -90 * LONGLAT_RATIO,
                                               180 * LONGLAT_RATIO, 90 * LONGLAT_RATIO};
            const unsigned char gridOffsetBit[10]{12, 10, 8, 6, 4, 2, 0, 0, 0, 0};
            const int xRatio = LONGLAT_RATIO;
            const int yRatio = LONGLAT_RATIO;
            const float DBL_EPSILON_OWN = 0.0000001f;
            const char levelChar[7] = {'X', 'A', 'C', 'D', 'F', 'H', 'X'};
            const long levelLonSize[7] = {0, 21600000, 5400000, 1800000,
                                          450000, 112500, 0};
            const long levelLatSize[7] = {0, 14400000, 3600000, 1200000,
                                          300000, 75000, 0};
            const unsigned char levelGridCount[7] = {0, 1, 4, 12, 48, 192, 0};
            //
            const int customLonSize[2] = {4500, 1500};
            const int customLatSize[2] = {3000, 1000};
        };
    }
}


#endif //AUTOHDMAP_GEO_MESHGRIDOBJECTEXT_HPP
