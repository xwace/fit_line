import os
import cv2
import math
import matplotlib.pyplot as plt
import numpy as np
import random

def get_laser_pt(points):
    ppts = []
    distance = []
    robot_angle = 0
    for i in range(len(points)):
        pts = points[i].strip().split(' ')
        if float(pts[0]) == 0 or float(pts[1]) == 0: continue
        robot_angle = float(pts[-1])
        angle = float(pts[0])/float(180) * math.pi
        x = math.cos(angle) * float(pts[1])
        y = math.sin(angle) * float(pts[1])
        ppts.append([x, y])
        distance.append(float(pts[1]))
    sorted_id = sorted(range(len(distance)), key=lambda k:distance[k])
    return np.asarray(ppts), robot_angle, sorted_id

def get_laser_pt1(points):
    # 保存连续的激光点，二维数组
    laser_points = []
    ppts = []
    num = 0
    last_laser = 0
    min_dist = 10000
    min_dists = []

    for i in range(len(points)):
        pts = points[i].strip().split(' ')
        if float(pts[0]) == 0 or float(pts[1]) == 0:
            num += 1
        else:
            num = 0

        if num > 5 or i == len(points)-1:
            if len(ppts) > 0:
                laser_points.append(ppts[:])
                min_dists.append(min_dist)
                min_dist = 10000
                ppts.clear()
                last_laser = 0
                num = 0
            continue
        if num != 0: continue
        num = 0
        angle = float(pts[0]) / float(180) * math.pi
        x = math.cos(angle) * float(pts[1])
        y = math.sin(angle) * float(pts[1])

        # plt.scatter(x, y)
        # plt.show()

        if last_laser == 0:
            last_laser = float(pts[1])
            continue
        if abs(float(pts[1]) - last_laser) > 100.0:  # 两边夹角约1度，因此 对边的长度约两边的长度差
            laser_points.append(ppts[:])
            ppts.clear()
            last_laser = 0
            min_dists.append(min_dist)
            min_dist = 10000
            continue
        last_laser = float(pts[1])
        ppts.append([x, y])
        if min_dist > float(pts[1]):
            min_dist = float(pts[1])

        # temp = np.asarray(ppts)
        # plt.scatter(temp[:,0], temp[:, 1])

        # plt.show()

    # 防止首尾存在相邻的点云被零号点分隔开
    # if len(laser_points) > 1:
    #     num = 0
    #     last1 = 0
    #     last2 = 0
    #     for i in range(len(points)):
    #         pts = points[i].strip().split(' ')
    #         if float(pts[0]) == 0 or float(pts[1]) == 0:
    #             num += 1
    #         else:
    #             last1 = float(pts[1])
    #             break
    #     for i in range(len(points)):
    #         i = len(points) - 1 - i
    #         pts = points[i].strip().split(' ')
    #         if float(pts[0]) == 0 or float(pts[1]) == 0:
    #             num += 1
    #         else:
    #             last2 = float(pts[1])
    #             break

        # if num < 2 and abs(last2 - last1) < 100.0:
        #     laser_points_temp = laser_points[1:]
        #     laser_points_temp[-1] += laser_points[0]
        #     laser_points = laser_points_temp
        #     min_dists_temp = min_dists[1:]
        #     min_dists_temp[-1] = min(min_dists[0], min_dists[-1])
        #     min_dists = min_dists_temp

    sorted_id = sorted(range(len(min_dists)), key=lambda k: min_dists[k])

    laser_points = [laser_points[i] for i in sorted_id]

    return laser_points
    pass


def leastSquare(points, pIdx, dist, tmp_ptsIdx):
    # 求解给定若干点的直线方程
    # a: x之和　b: y之和  c: x平方和  d: x*y之和  e: 样本数量
    a = 0
    B = 0
    c = 0
    d = 0
    s = len(pIdx)
    # 通过4个点求得斜率
    # cur_ptsIdx = [random.randint(1, s//2) for i in range(4)]
    cur_ptsIdx = []
    for i in range(5):
        k = random.randint(1, s//2)
        if k not in cur_ptsIdx:
            cur_ptsIdx.append(k)
    e = len(cur_ptsIdx)
    for j in range(e):
        i = pIdx[cur_ptsIdx[j]]
        a += points[i][0]
        B += points[i][1]
        c += points[i][0] * points[i][0]
        d += points[i][0] * points[i][1]

    k = 1
    b = 0
    tmp = e*c - a*a
    if abs(tmp) > 0.0005:
        b = (c*B - a*d)/tmp
        k = (e*d - a*B)/tmp

    # 求每个点到直线的距离，小于dist，即在直线上
    line_pnum = 0
    dist_num = 0
    for i in pIdx:
        # 分子分母  点到直线的距离　d = |kx - y + b| / sqrt(k^2 + 1)
        numerator = abs(k*points[i][0] - points[i][1] + b)
        denominator = math.sqrt(k * k + 1)
        d = numerator / denominator
        if d < dist:
            line_pnum += 1
            dist_num += d
            tmp_ptsIdx.append(i)
    line_model = [k, b, line_pnum, dist_num]
    return line_model, tmp_ptsIdx

# 点集合、点到直线的距离、迭代次数
def ransac_line(points, dist, iterate):
    cur_ptsIdx = list(range(len(points)))
    lines = []
    line_model = []
    best_lineModel = []
    r = 0
    tmp_ptsIdx = []
    line_ptsIdx = []
    cur_line = [0, 0]
    while True:
        line_pts = 0
        tmp_pts = 0
        if r >= 2: iterate = iterate/3
        if len(cur_ptsIdx) < 10 and len(cur_ptsIdx) > 3: iterate = 4;
        for i in range(iterate):
            line_model, tmp_ptsIdx = leastSquare(points, cur_ptsIdx, dist, tmp_ptsIdx)
            tmp_pts = line_model[2] / 1
            if tmp_pts > line_pts:
                line_pts = tmp_pts
                best_lineModel = line_model
                line_ptsIdx = tmp_ptsIdx[:]
            tmp_ptsIdx.clear()
        cur_line[0] = best_lineModel[0]
        cur_line[1] = best_lineModel[1]
        lines.push_back(cur_line)
        print("第 ", r, " 次循环，直线参数：", best_lineModel)

        for i in range(len(line_ptsIdx)):
            pass


def get_line(points):
    cur_ptsIdx = list(range(len(points)))
    lines = []
    for k in range(4):
        tmp_pts = 0
        k = 1
        b = 0
        ptsIdx = []
        is_update = False
        if len(cur_ptsIdx) < 7: break
        for i in range(30):
            tmp_ptsIdx = []
            line_model, tmp_ptsIdx = leastSquare(points, cur_ptsIdx, 7, tmp_ptsIdx)
            #tmp_pts = line_model[2]
            if line_model[2] > 3 and tmp_pts < line_model[2]:
                tmp_pts = line_model[2]
                k = line_model[0]
                b = line_model[1]
                ptsIdx = tmp_ptsIdx[:]
                is_update = True

        if is_update: lines.append([k, b])
        else: break

        remain_ptIdx = []
        for i in cur_ptsIdx:
            if i not in ptsIdx:
                remain_ptIdx.append(i)

        cur_ptsIdx = remain_ptIdx[:]
        if len(cur_ptsIdx) < 7:
            break
        # break

    return lines
    # plt.plot(0, 0, marker='+', color='coral')
    # pts = np.asarray(points)
    # plt.scatter(pts[:, 0], pts[:, 1])
    # x = np.linspace(pts[0][0], pts[-1][0], 60)
    # y1 = k*x + b
    # plt.plot(x, y1)
    # plt.show()
    # pass

if __name__ == "__main__":
    with open("alongwallLaser_test.txt", "r") as file:
        lines = file.readlines()

    gap_id = [i for i in range(len(lines)) if lines[i].strip() == "-1 -1 -1 -1 -1 -1"]

    for i in range(128, len(lines)-1):
        print(i)
        start_id = gap_id[i]+1
        end_id = gap_id[i+1]
        line = lines[start_id:end_id]
        # ppts, robot_angle, sorted_id = get_laser_pt(line)
        ppts = get_laser_pt1(line)
        for pts in ppts:
            if len(pts) < 5: continue
            pts = np.asarray(pts)
            plt.scatter(pts[:, 0], pts[:, 1])
            kbs = get_line(pts)
            x = np.linspace(min(pts[:, 0]), max(pts[:, 0]), 60)
            for kb in kbs:

                y1 = kb[0]*x + kb[1]
                plt.plot(x, y1, color='black')

        plt.plot(0,0, marker='+', color='coral')

        # for pts in ppts:
        #     if len(pts) < 5: continue
        #     pts = np.asarray(pts)
        #     plt.scatter(pts[:,0], pts[:,1])
        plt.show()


        pass