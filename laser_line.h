//
// Created by wdh on 2023/1/6.
//

#ifndef MAP_FUSION2_LASER_LINE_H
#define MAP_FUSION2_LASER_LINE_H
#include <vector>
struct Point
{
    float x, y; // xy为坐标系
    int id; // id对应的点编号
    Point(float a, float b, int c): x(a), y(b), id(c){} // 有参数的构造
};

struct Line
{
    float k, b, length;  // 线段的斜率、截距、与原点的距离
    int start_id, end_id;  // 线段拟合的点集索引
    int target_id;  // 90度 或 180度的标志位
    float angle;    // 拟合出来的直线与X轴（机器人的正方向）的 夹角，返回值为角度(0~90度)
    Line(float a, float c, float d, int e, int f, int g, float h): k(a), b(c), length(d), start_id(e), end_id(f), target_id(g), angle(h){}
};

void get_line(); // 调用txt文档的测试函数接口

/** 注：对传入数据结构，可以参考 laser_line.cpp/get_laser_pt(std::string& file_name, std::vector<std::vector<Point>>& points) 函数
 * 对一帧雷达数据中90度和180度范围内的点集进行直线拟合，返回拟合后的结果
 * @param points 雷达数据，points中一个Point数据的 x 为 laser.mLaserData[i].nAngle， y 为 laser.mLaserData[i].nDist_MM， id为雷达点的索引编号即 i
 * @param lines 返回拟合后的90度和180度线段
 */
void get_laser_line(std::vector<Point>& points, std::vector<Line>& lines);  // 对外调用接口
#endif //MAP_FUSION2_LASER_LINE_H
