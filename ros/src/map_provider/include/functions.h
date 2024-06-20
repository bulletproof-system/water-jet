#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "ros/ros.h"
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"

// rdm 类，用于生成随机浮点数
class rdm {
    int i;
public:
    rdm();
    float randomize();
};

// Norm 函数原型，计算两个点之间的欧几里得距离
float Norm(std::vector<float>, std::vector<float>);

// sign 函数原型，返回数值的符号
float sign(float);

// Nearest 函数原型，找到距离 x 最近的点
std::vector<float> Nearest(std::vector<std::vector<float>>, std::vector<float>);

// Steer 函数原型，计算新的点
std::vector<float> Steer(std::vector<float>, std::vector<float>, float);

// gridValue 函数原型，返回 "Xp" 位置的网格值
int gridValue(nav_msgs::OccupancyGrid&, std::vector<float>);

// ObstacleFree 函数原型，检查路径是否无障碍
char ObstacleFree(std::vector<float>, std::vector<float>&, nav_msgs::OccupancyGrid);

#endif // FUNCTIONS_H
