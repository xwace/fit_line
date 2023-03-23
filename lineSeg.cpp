/**
 * description:输入点集，聚类生成多个线段，提取低于地面的悬崖线段，返回悬崖高度
 * @author oswin
 * @date 2023/3/23
 */
float keypoints(const vector<Point2f>&points) {

    vector<Range> lines;
    vector<double> crossResults((int) points.size(), 0);
    int thres_cross = 100;//判断共线
    float thres_h = 290;//判断是否低于地面
    float angle_thres = 0.523599;//tan(30度)
    #define min(a, b) (a<b ? a:b)

    for (int i = 1; i < (int) points.size() - 1; ++i) {
        auto prev = points[i - 1];
        auto cur  = points[i];
        auto next = points[i + 1];
        auto a    = cur - prev;
        auto b    = next - cur;

        //叉积判断共线
        auto c = crossResults[i] = (a).cross(b);

        auto pushLineSeg = [&](vector<Range>& lines) {
            int j = i;
            while (--j > 0 and crossResults[j] < thres_cross) {}
            if (i - j > 1)
                lines.emplace_back(j + 1, i - 1);
        };

        if (fabs(c) > thres_cross) {
            if (cur.y > thres_h)
                pushLineSeg(lines);
        }

        if (i == (int) points.size() - 2) {
            pushLineSeg(lines);
        }
    }

    for (auto &line: lines) {

        //筛选掉不平行地面的线段
        auto start = points[line.start];
        auto end = points[line.end];
        auto vect = end - start;

        if (std::fabs(atan2(vect.y, vect.x)) < angle_thres
            && start.y > thres_h and end.y > thres_h) {
            LOGD("Cliff detected!!!");
            return min(start.y, end.y);//返回悬崖线段
            #undef min
        }
    }

    LOGD("No cliff detected!!!");
    return -1;
}
