/*
2d平面碰撞检测，使用分离轴算法，用于检测两个凸多边形是否相交
项目中实际使用判断矩形框是否在ROI区域中
*/

#include <cstdio>
#include <vector>
#include <climits>

struct Vec {
    int x;
    int y;

    Vec() : x(0), y(0) {}

    Vec(int _x, int _y): x(_x), y(_y) {}

    /// @brief  求法向量
    /// @return 法向量
    Vec Normal()
    {
        return Vec(y, -x);
    }

    /// @brief 向量点积 a*b
    /// @param rhs b向量
    /// @return  b在a向量上的投影长度
    int operator *(const Vec &rhs)
    {
       return x * rhs.x + y * rhs.y; 
    }
};

struct Point {
    int x;
    int y;

    Point(): x(0), y(0) {}

    Point(int _x, int _y): x(_x), y(_y) {}

    Vec operator -(const Point &rhs)
    {
        Vec v;
        v.x = x - rhs.x;
        v.y = y - rhs.y;
        return v;
    }
    Vec toVec() const
    {
        return Vec(x, y); 
    }
};

struct Rect {
    int left;
    int top;
    int width;
    int height;
};

std::vector<Point> rect_to_points(const Rect &rect)
{
    std::vector<Point> points;
    points.push_back({rect.left, rect.top});
    points.push_back({rect.left + rect.width, rect.top});
    points.push_back({rect.left + rect.width, rect.top + rect.height});
    points.push_back({rect.left, rect.top + rect.height});

    return points;
    
}

/// @brief 判断矩形是否与roi区域相交
/// @param roi 检测区域
/// @param rect 检测框
/// @return true:相交 false:不想交
bool collision_detect(const std::vector<Point> &roi, const Rect &rect)
{   
    if (roi.size() < 3) {
        printf("roi points must >= 3\n");
        return false; 
    }
    /// 首先可以拿到roi包围盒，判断两个矩形是否相交，加快速度
    int x1 = INT_MAX, y1 = INT_MAX;
    int x2 = INT_MIN, y2 = INT_MIN;
    for (const auto &p : roi) {
        x1 = std::min(x1, p.x);
        y1 = std::min(y1, p.y);
        x2 = std::max(x2, p.x);
        y2 = std::max(y2, p.y);
    } 
    /// 如果两个矩形都未相交，那么rect与roi肯定不会相交
    /// 两个矩形若相交，还得进一步判断
    if (rect.left > x2 || (rect.left + rect.width) < x1 ||
        rect.top > y2 || (rect.top + rect.height) < y1) {
        return false;        
    }
    std::vector<Point> rec_points = rect_to_points(rect);

    /// separating axis
    /// 1. 计算ROI, rect每一条边的法向量
    /// 2. 将ROI所有点、rect所有点投影到该法向量
    /// 3. 判断分离轴是否存在： 1）存在：说明不想交，直接返回 2）不存在：继续寻找分离轴
    /// 4. 循环1-3步骤，如果结束还没找到分离轴，说明两个区域相交

    /// 计算roi所有法向量 + rect两个法向量
    int project_cnt = roi.size() + 2;
    for (size_t i = 0; i < project_cnt; ++i) {
        Point p0, p1;
        if (i < roi.size()) {
            p0 = roi[i];
            p1 = roi[(i+1)%roi.size()];
        }
        else { /// rect 
            int idx = project_cnt - roi.size();
            p0 = rec_points[idx];
            p1 = rec_points[idx+1];
        }

        Vec v = p1 - p0;
        Vec normal = v.Normal();

        /// 计算roi rect 所有点在法向量上投影：实际上所有点和（0，0）组成的向量在法向量上的投影
        /// @note 向量点乘可能为负数，pj_max初始值不能取0 
        int roi_pj_min = INT_MAX, roi_pj_max = INT_MIN;
        int rec_pj_min = INT_MAX, rec_pj_max = INT_MIN;

        for (auto &p : roi) {
            int len = normal * (p.toVec());
            roi_pj_min = std::min(roi_pj_min, len);
            roi_pj_max = std::max(roi_pj_max, len);
        }


        for (auto &p : rec_points) {
            int len = normal * (p.toVec());
            rec_pj_min = std::min(rec_pj_min, len);
            rec_pj_max = std::max(rec_pj_max, len);
        }

        /// 存在分离轴，直接返回不相交
        if (rec_pj_min > roi_pj_max || rec_pj_max < roi_pj_min) {
            return false;
        }
    }

    /// 遍历结束找不到分离轴，相交
    return true;
}


int main(int argc, char **argv)
{
    std::vector<Point> roi{Point(200, 0), Point(200, 200), Point(0, 200)};
    
    Rect rect1 = {
        .left = 0,
        .top = 0,
        .width = 100,
        .height = 100,
    };


    Rect rect2 = {
        .left = 50,
        .top = 50,
        .width = 40,
        .height = 40,
    };

    Rect rect3 = {
        .left = 201,
        .top = 101,
        .width = 50,
        .height = 50,
    };

    Rect rect4 = {
        .left = 180,
        .top = 100,
        .width = 50,
        .height = 50,
    };

    bool ret1 = collision_detect(roi, rect1);
    bool ret2 = collision_detect(roi, rect2);
    bool ret3 = collision_detect(roi, rect3);
    bool ret4 = collision_detect(roi, rect4);
    printf("collision detect, rect1 and roi:%s, rect2 and roi:%s\n",
            ret1 ? "collision" : "no collision", ret2 ? "collision": "no collision");

    printf("collision dedect ,rec3 and roi:%s, rect4 and roi:%s\n",
            ret3 ? "collision" : "no collision", ret4 ? "collision": "no collision"); 
    return 0;
}
