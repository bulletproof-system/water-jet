#!/usr/bin/env python
# coding=utf-8

# -------- 导入模块 ---------------
import cv2
import numpy as np
from copy import copy


# 获取前沿点函数
def get_frontier(mapData):
    data = mapData.data
    w = mapData.info.width
    h = mapData.info.height
    resolution = mapData.info.resolution
    X_startx = mapData.info.origin.position.x
    X_starty = mapData.info.origin.position.y

    # 创建空白图像
    img = np.zeros((h, w, 1), np.uint8)

    # 填充图像数据
    for i in range(h):
        for j in range(w):
            if data[i * w + j] == 100:
                img[i, j] = 0
            elif data[i * w + j] == 0:
                img[i, j] = 255
            elif data[i * w + j] == -1:
                img[i, j] = 205

    # 阈值处理
    o = cv2.inRange(img, 0, 1)
    edges = cv2.Canny(img, 0, 255)
    im2, contours, hierarchy = cv2.findContours(o, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(o, contours, -1, (255, 255, 255), 5)
    o = cv2.bitwise_not(o)
    res = cv2.bitwise_and(o, edges)

    # 复制结果并找到前沿点
    frontier = copy(res)
    im2, contours, hierarchy = cv2.findContours(frontier, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frontier, contours, -1, (255, 255, 255), 2)
    im2, contours, hierarchy = cv2.findContours(frontier, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    all_pts = []
    if len(contours) > 0:
        for i in range(len(contours)):
            cnt = contours[i]
            M = cv2.moments(cnt)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            xr = cx * resolution + X_startx
            yr = cy * resolution + X_starty
            pt = [np.array([xr, yr])]
            if len(all_pts) > 0:
                all_pts = np.vstack([all_pts, pt])
            else:
                all_pts = pt

    return all_pts
