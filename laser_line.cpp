//
// Created by wdh on 2023/1/6.
//

#include "laser_line.h"
#include <iostream>
#include <fstream>

#include <cmath>
#include <algorithm>
#include <ctime>
// matplotlibcpp 是实现在CPP中调用python的画图模块matplotlib
// 使用参考 https://github.com/Cryoris/matplotlib-cpp
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

void get_laser_pt(std::string& file_name, std::vector<std::vector<Point>>& points)
{
    std::ifstream file_txt(file_name); // 创建文件流读对象
    std::vector<std::string> txt;
    std::string s; // 保存读取的梅一行数据

    std::vector<Point> pts;
    pts.reserve(361);
    std::string::size_type pos1;
    std::string::size_type pos2;
    while(std::getline(file_txt, s))
    {
        if (s == "-1 -1 -1 -1 -1 -1")
        {
            points.push_back(std::move(pts));
            pts.clear();
            pts.reserve(361);
            continue;
        }

//        s = "252.8 583.868 -0.0998091 -0.435927 -0.0100806 -6.5379"; // 测试字符分割的效果

        pos1 = s.find_first_of(' ', 0);
        float angle = stof(s.substr(0, pos1));
        pos2 = s.find_first_of(' ', pos1+1);
        float length = stof(s.substr(pos1, pos2-pos1));
        int id = (int)pts.size();
        pts.push_back(Point(angle, length, id));
    }
}

void draw(std::vector<Point>& points, std::vector<Line>& lines)
{
    int n = (int)points.size();
    std::vector<double> x(n), y(n);
    for (int i = 0; i < n; i++)
    {
        x[i] = points[i].x;
        y[i] = points[i].y;
    }
//    plt::xkcd();
    plt::scatter(x, y);
    std::vector<double> y1(n);
//    std::iota(x1.begin(), x1.end(), x[0]);
    for (Line l : lines)
    {
        for (int i = 0; i < n; i++)
        {
            y1[i] = l.k * x[i] + l.b;
        }
        plt::plot(x, y1);
    }

    plt::show();
}

void drawss(std::vector<std::vector<Point>>& pointss, std::vector<std::vector<Line>>& liness)
{
    for (int i = 0; i < pointss.size(); i++)
    {
        auto points = pointss[i];
        int n = (int)points.size();
        std::vector<double> x(n), y(n);
        for (int i = 0; i < n; i++)
        {
            x[i] = points[i].x;
            y[i] = points[i].y;
        }
        plt::scatter(x, y);
//        std::vector<double> y1(n);
//        std::vector<Line> lines = liness[i];
//        for (Line l : lines)
//        {
//            for (int i = 0; i < n; i++)
//            {
//                y1[i] = l.k * x[i] + l.b;
//            }
//            plt::plot(x, y1);
//        }
    }


    plt::show();
}

void draw_line(std::vector<Point>& points, std::vector<Line>& lines)
{
    int n = (int)points.size();
    std::vector<double> x(n+1), y(n+1);
    int id90 = 0, id180 =  0;
    for (int i = 0; i < n; i++)
    {
        Point pt = points[i];
        x[i] = pt.x;
        y[i] = pt.y;
        if (pt.id == 90) id90 = i;
        else if (pt.id == 180) id180 = i;
    }
    x[n] = 0.0;
    y[n] = 0.0;

//    plt::xkcd();
    plt::scatter(x, y);
    std::vector<double> x1(2), y1(2);
    x1[0] = points[id90].x;
    y1[0] = points[id90].y;

    x1[1] = points[id180].x;
    y1[1] = points[id180].y;

//    plt::plot(x1, y1);

    for (Line l : lines)
    {
        std::vector<double> y3, x3;
        int start_id, end_id;
        for (int i = 0; i < n; i++)
        {
            if (points[i].id == l.start_id) start_id = i;
            if (points[i].id == l.end_id) end_id = i;
        }
        x3.push_back(points[start_id].x);
        x3.push_back(points[end_id].x);

        y3.push_back(l.k * points[start_id].x + l.b);
        y3.push_back(l.k * points[end_id].x + l.b);
        plt::plot(x3, y3);
    }

    plt::show();
}

template <typename T>
std::vector<int> sort_indexes_e(std::vector<T>& v)
{
    std::vector<int> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0); // 以 value 开始，顺序填充值
    std::sort(idx.begin(), idx.end(), [&v](int i1, int i2) {return v[i1] < v[i2];});
    return idx;
}

/**
 * 将一帧雷达点 分割成 多个线段点集，方便后面的直线拟合
 * @param pts 雷达点集
 * @param line_pts 线段点集first，距离second
 */
void get_line_pt(std::vector<Point>& points, std::vector<std::pair<std::vector<Point>, float>>& line_pts)
{
    int start_id = 45, end_id = 225; // 雷达角度范围内的点
    int num = 0; // 统计不连续点的个数
    int total_num = 5; // 超过5个点，则认为是一个线段点集

    float last_laser = 0.0f; // 保存上一个雷达点线段长度
    float laser_diff = 300.0f;
    bool is_update = false;  // 是否保存线段点集的标志

    float min_dist = 10000.0f; // 保存每个线段点集距离雷达最近的长度
    std::vector<float> min_dists;

    float angle2rad = M_PI / 180.0f; // 后续计算由除法改为乘法，提高运算效率

    std::vector<Point> pts; // 保存一个线段的点集
    pts.reserve(end_id-start_id+1);  // 先开辟内存，提高代码运行效率
    for (size_t i = start_id; i < end_id; i++)
    {
        Point pt = points[i];
        if (pt.x == 0.0f || pt.y == 0.0f) num += 1;
        else num = 0;

        if (num > total_num) is_update = true;

        if (!is_update && num == 0) // 说明可以加入当前点
        {
            pt.x = pt.x * angle2rad;  // 由角度转换为弧度
            // 将极坐标 转换 为迪卡尔坐标，x轴朝上，y轴朝左
            float x = std::sin(pt.x) * pt.y;
            float y = -std::cos(pt.x) * pt.y;
            Point ppt(x, y, pt.id);

            if (min_dist > pt.y) min_dist = pt.y;  // 保存线段点集中距离雷达最近的线段长度
            if (last_laser == 0.0f) last_laser = pt.y; // 保存雷达线段长度
            if (std::abs(pt.y - last_laser) > laser_diff) is_update = true;
            else pts.push_back(ppt);
        }

        if (i == end_id-1) is_update = true;

        if (is_update && !pts.empty()) // 说明需要更新保存线段点集
        {
            line_pts.emplace_back(std::make_pair(std::move(pts), min_dist)); // 保存线段点集
            pts.clear(); // 临时点集清空
            num = 0;
            last_laser = 0.0f;
            is_update = false;
            min_dists.push_back(min_dist); // 保存线段点集的最短长度
            min_dist = 10000.0f;
        }
    }

//    if (line_pts.size() > 1) // 只有超过1个线段集才进行距离排序赋值
//    {
//        // 对距离进行排序，获取排序后的索引
//        std::vector<int> idx = sort_indexes_e(min_dists);
//        // 根据排序后的索引，对line_pts进行排序
//        std::vector<std::pair<std::vector<Point>, float>> line_temp(line_pts.size());
//        for (int i = 0; i < idx.size(); i++) line_temp[i] = (std::move(line_pts[idx[i]]));
//        line_pts = std::move(line_temp);
//        std::vector<std::pair<std::vector<Point>, float>>().swap(line_temp);
//        std::vector<int>().swap(idx);
//    }

    std::vector<float>().swap(min_dists);
    std::vector<Point>().swap(pts);
}

/**
 * 获取当前的系统时间，作为随机数的种子
 * @return 返回当前系统的时间，单位为ms
 */
unsigned long GetTickCount()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC,&ts);
    return(ts.tv_sec*1000+ts.tv_nsec/1000000);
}

/**
 * 通过RASCAN算法，获取点集的拟合直线，距离直线满足要求的点数最多的则为最终的直线
 * @param points 目标点集
 * @param pIdx 目标点集拟合直线的点索引
 * @param new_pIdx 满足要求的点索引
 * @param line 拟合的直线
 * @param dist 与直线的距离阈值
 * @param start_id 目标点集拟合直线的点索引 的 起点索引，用于获取随机点索引
 * @param end_id 目标点集拟合直线的点索引 的 终点索引，用于获取随机点索引
 * @return 返回满足要求的点与直线的距离和
 */
float least_square(std::vector<Point>& points, std::vector<int> &pIdx, std::vector<int> &new_pIdx, Line& line, float dist, int start_id, int end_id)
{
    int t = end_id - start_id;  // 点数
    int m = 5; // 设置随机采样5个点计算直线的斜率和截距
    if (t < 5) m = t; // 小于5个点，则采用对应的点
    std::vector<int> cur_ptsIdx;
    cur_ptsIdx.reserve(m);
    static bool is_init = false;
    if (!is_init)
    {
        //设置时间种子，若不设置随机种子，则每次运行函数产生的随机数是一样的
        // 这函数不能反复调用，只能在程序启动的时候才能调用
        srand(GetTickCount());
        is_init = true;
    }

    for (int i = 0; cur_ptsIdx.size() < m; i++)
    {
        int idx = std::rand()%(end_id-start_id)+start_id;  // 生成区间 start_id ～ end_id 的随机数，并保证生成的随机数不能重复。
        if (std::find(cur_ptsIdx.begin(), cur_ptsIdx.end(), idx) == cur_ptsIdx.end()) cur_ptsIdx.push_back(idx);
    }

    // 根据获得的随机点，求得随机点组成线段的斜率k和截距b
    float a = 0.0f, B = 0.0f, c = 0.0f, d = 0.0f; // a: x之和　b: y之和  c: x平方和  d: x*y之和
    int e = (int)cur_ptsIdx.size();
    for (int i = 0; i < e; i++)
    {
        int j = cur_ptsIdx[i];   // 获取随机数值
        Point pt = points[j];
        a += pt.x;
        B += pt.y;
        c += pt.x * pt.x;
        d += pt.x * pt.y;
    }

    // 计算直线的斜率k和截距b
    float k = 1.0f, b = 0.0f;
    float tmp = e*c - a*a;
    if (std::abs(tmp) > 0.0005f)
    {
        b = (c*B - a*d)/ tmp;
        k = (e*d - a*B) / tmp;
    }

    float dist_num = 0.0;
    // 求每个点到直线上的距离，小于dist即在直线上，则保存在new_pIdx向量中
    for (int i = 0; i < (int)pIdx.size(); i++)
    {
        int j = pIdx[i];
        Point pt = points[j];
        // 点到直线的距离　d = |kx - y + b| / sqrt(k^2 + 1)
        float numerator = std::abs(k*pt.x - pt.y + b);
        float denominator = std::sqrt(k*k + 1);
        d = numerator / denominator;
        if (d < dist)
        {
            new_pIdx.push_back(j);
            dist_num += d;
        }
    }
    // 保存直线的斜率k和截距b
    if (new_pIdx.size() > 4)
    {
        line.k = k;
        line.b = b;
        line.length = std::abs(b) / std::sqrt(k*k + 1); // 保存当前线段距离(0,0) 的距离

        line.start_id = points[new_pIdx.front()].id; // 拟合直线的线段点编号起点
        line.end_id = points[new_pIdx.back()].id; // 拟合直线的线段点编号终点
        // 因将雷达点的坐标系转换为 前x， 左y。而计算的直线斜率还是按照 前y右x的方式，因此需要求与雷达x轴夹角，相当于 前y右x的方式 与y轴的夹角
        line.angle = 90.0f - std::abs(std::atan(k) * M_PI / 180.0f);  // atan返回结果为 -90 ～ 90 度
    }
    std::vector<int>().swap(cur_ptsIdx);
    return dist_num;
}

/**
 * 通过循环迭代的方式，不断拟合直线，直到找到满足要求的直线为止
 * @param points 待拟合直线的点集
 * @param lines 拟合出来的直线
 */
void ransac_line(std::vector<Point>& points, std::vector<Line>& lines)
{
    int num = (int)points.size();
    if (num <= 4) return;  // 点集小于4个点，则不进行直线拟合
    // 获取当前点集的索引编号
    std::vector<int> idx(num);
    std::iota(idx.begin(), idx.end(), 0); // 以 value 开始，顺序填充值

    int iterate = num/2;  // 根据点数设置迭代此次，最大的迭代次数为30
    if (iterate > 30) iterate = 30;

    float dist = 6.0f; // 点到拟合直线的距离阈值为6毫米
    int start_id = 0, end_id = num-1; // 起始和截止点索引编号
    // 点集最大拟合次数为3次，因为点集中会存在多条直线，通过观察，45度到225度之间，满足要求的线段最多为3条
    for (int i = 0; i < 3; i++)
    {
        int pt_num = 0, count = 0;  // 记录拟合直线满足要求的点数，以及连续拟合多少次不满足要求直线的次数
        Line line(1.0f, 0.0f, 0.0f, 0, 0, -1, 1.0f); // 记录拟合的直线
        std::vector<int> new_pIdx_temp; // 保存单次拟合直线的点集索引
        std::vector<int> new_pIdx; // 保存迭代中满足要求的点集索引
        if (iterate > 30) iterate = 30; // 设置最大的迭代次数
        // 不停的迭代，找到符合要求的直线
        for (int j = 0; j < iterate; j++)
        {
            if (count > iterate/2) break;  // 连续多次未找到满足要求的直线，则退出当前的迭代
            Line line_tmp(1.0f, 0.0f, 0.0f, start_id, end_id, -1, 1.0f); // 当前拟合出的直线
            least_square(points, idx, new_pIdx_temp, line_tmp, dist, start_id, end_id);
            int size = (int)new_pIdx_temp.size();
            // 在此次迭代中，找出拟合点数最多的直线，拟合的直线满足的点数小于4个，则认为直线不合法要求
            if (size > 4 && pt_num < size)
            {
                pt_num = size;
                line = line_tmp;
                new_pIdx = std::move(new_pIdx_temp);
                count = 0;
            }
            else count += 1; // 不符合要求的直线次数统计
            new_pIdx_temp.clear();
        }
        // 没有拟合得到直线，则退出当前循环
        if (pt_num == 0) break;
        lines.push_back(line); // 保存拟合的直线

        if (idx.size()-new_pIdx.size() < 5) break;

        // 获取拟合直线的点集以外的点集，进行下一次直线拟合
        std::vector<int> remain_pIdx;
        for (int & k : idx) if (std::find(new_pIdx.begin(), new_pIdx.end(), k) == new_pIdx.end()) remain_pIdx.push_back(k);

        if (remain_pIdx.size() < 5) break;  // 剩余的点数小于5,则不进入下一次直线拟合

        // 判断剩余点的连续的连续点中起始和终止索引号
        start_id = remain_pIdx.front(), end_id = remain_pIdx.back();
        int tmp_id = -1, length_ids = 0; // 记录起点，和点索引差值
        remain_pIdx.push_back(remain_pIdx[0]); // 在最后保存起点索引，主要为了方便最后一次获取最后一次操作
        num = (int)remain_pIdx.size();
        for (int k = 0; k < num - 1; k++)
        {
            int id1 = remain_pIdx[k];
            int id2 = remain_pIdx[k+1];
            int diff = std::abs(id2 - id1); // 记录相邻点索引差值
            if (tmp_id == -1 && diff < 5) tmp_id = id1; // 记录起点索引
            else if (tmp_id != -1 && diff >= 5) // 记录到终止索引
            {
                if (length_ids < id1-tmp_id) // 找到索引差值最大的起始和终止索引
                {
                    length_ids = id1-tmp_id;
                    start_id = tmp_id;
                    end_id = id1;
                }
                tmp_id = -1;
            }
        }

        if (end_id - start_id < 5) break;  // 剩余的点数小于5,则不进入下一次直线拟合

        // 将得到的新索引赋值给idx，进行下一次直线拟合迭代
        idx.clear();
        idx.insert(idx.end(), remain_pIdx.begin(), remain_pIdx.end()-1);
        std::vector<int>().swap(remain_pIdx);
        iterate = (int)idx.size()/2;
    }
}

/**
 * 获取雷达数据中，90度和180度方向上的线段
 * @param line_pts 分段后的雷达数据点
 * @param lines 90度和180度方向上的线段
 */
void line_fit(std::vector<std::pair<std::vector<Point>, float>>& line_pts, std::vector<Line> lines)
{
//    std::vector<Point> pointss;
//    std::vector<std::vector<Line>> liness;
    int num = (int)line_pts.size();
    int id90 = -1, id180 = -1;
    // 查找出180度和90度所在的点集 编号
    for (int i = 0; i < num; i++)
    {
        auto line_pt = line_pts[i];
//        pointss.insert(pointss.end(), line_pt.first.begin(), line_pt.first.end());
        if (line_pt.second > 500.0f) continue; // 大于50厘米则不进行直线拟合
        if ((line_pt.first.front().id <= 90 && line_pt.first.back().id >= 90)) id90 = i;   // 90度在范围内
        else if (i < num-1 && (line_pts[i].first.back().id < 90 && line_pts[i+1].first.front().id > 90)) // 离90度最近的编号
        {
            int diff1 = 90 - line_pts[i].first.back().id;
            int diff2 = line_pts[i+1].first.front().id - 90;
            if (diff1 > 5 && diff2 > 5) continue;
            if (diff1 < diff2) id90 = i;
            else id90 = i+1;
        }
        else if ((line_pt.first.front().id <= 180 && line_pt.first.back().id >= 180)) id180 = i;  // 180度范围内
        else if (i < num-1 && (line_pts[i].first.back().id < 180 && line_pts[i+1].first.front().id > 180)) // 离180度最近的编号
        {
            int diff1 = 180 - line_pts[i].first.back().id;
            int diff2 = line_pts[i+1].first.front().id - 180;
            if (diff1 > 5 && diff2 > 5) continue;
            if (diff1 < diff2) id180 = i;
            else id180 = i+1;
        }
    }

    if (id90 != -1 && id90 == id180) // 则说明90度和180度
    {
        std::vector<Line> ll;
        auto line_pt = line_pts[id90];
        ransac_line(line_pt.first, ll);
        num = (int)ll.size();
        id90 = -1, id180 = -1;
        // 找出 90 度 和 180度所在的直线
        for (int i = 0; i < num; i++)
        {
            Line l = ll[i];
            if (l.start_id <= 90 && l.end_id >= 90) id90 = i;
            else if (l.start_id <= 180 && l.end_id >= 180) id180 = i;
        }
        if (id90 != -1)
        {
            ll[id90].target_id = 90;
            lines.push_back(ll[id90]);
        }
        if (id180 != -1)
        {
            ll[id90].target_id = 180;
            lines.push_back(ll[id90]);
        }
    }
    if (id90 != -1) // 90度线段直线拟合
    {
        std::vector<Line> ll;
        auto line_pt = line_pts[id90];
        ransac_line(line_pt.first, ll);

        for (auto & l : ll)
        {
            if (l.start_id <= 90 && l.end_id >= 90)
            {
                l.target_id = 90;
                lines.push_back(l);
            }
        }
    }
    if (id180 != -1) // 180度线段直线拟合
    {
        std::vector<Line> ll;
        auto line_pt = line_pts[id180];
        ransac_line(line_pt.first, ll);
        for (auto & l : ll)
        {
            if (l.start_id <= 180 && l.end_id >= 180)
            {
                l.target_id = 180;
                lines.push_back(l);
            }
        }
    }

//    draw_line(pointss, lines);
}

/**
 * 对一帧雷达数据中90度和180度范围内的点集进行直线拟合，返回拟合后的结果
 * @param points 雷达数据，points中一个数据的 x 为 laser.mLaserData[i].nAngle， y 为 laser.mLaserData[i].nDist_MM， id为雷达点的索引编号即 i
 * @param lines 返回拟合后的90度和180度线段
 */
void get_laser_line(std::vector<Point>& points, std::vector<Line>& lines)
{
    std::vector<std::pair<std::vector<Point>, float>> line_pts;
    get_line_pt(points, line_pts); // 将一帧雷达数据分成几段
    line_fit(line_pts, lines);  // 对满足要求的90度和180度雷达数据点进行拟合
}

void get_line()
{
    std::string name_path = "alongwallLaser_test.txt";
    std::vector<std::vector<Point>> points;
    std::vector<Line> lll;
    get_laser_pt(name_path, points);

    for (int i = 138; i < (int)points.size(); i++)
    {
        std::cout << "#################################### " << i << std::endl;
        auto pts = points[i];
        std::vector<std::pair<std::vector<Point>, float>> line_pts;

        get_line_pt(pts, line_pts);
        std::vector<Line> lines;

        line_fit(line_pts, lines);

        std::cout << std::endl;
    }
}