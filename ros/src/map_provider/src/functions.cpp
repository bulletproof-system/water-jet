#include "functions.h"

// rdm 类，用于生成随机浮点数
rdm::rdm() {
    i = time(0);
}

float rdm::randomize() {
    i = i + 1;
    srand(i);
    return float(rand()) / float(RAND_MAX);
}

// Norm 函数，计算两个点之间的欧几里得距离
float Norm(std::vector<float> x1, std::vector<float> x2) {
    return pow(pow((x2[0] - x1[0]), 2) + pow((x2[1] - x1[1]), 2), 0.5);
}

// sign 函数，返回数值的符号
float sign(float n) {
    if (n < 0.0) {
        return -1.0;
    } else {
        return 1.0;
    }
}

// Nearest 函数，找到距离 x 最近的点
std::vector<float> Nearest(std::vector<std::vector<float>> V, std::vector<float> x) {
    float min = Norm(V[0], x);
    int min_index = 0;
    float temp;

    for (int j = 0; j < V.size(); j++) {
        temp = Norm(V[j], x);
        if (temp <= min) {
            min = temp;
            min_index = j;
        }
    }

    return V[min_index];
}

// Steer 函数，计算新的点
std::vector<float> Steer(std::vector<float> x_nearest, std::vector<float> x_rand, float eta) {
    std::vector<float> x_new;

    if (Norm(x_nearest, x_rand) <= eta) {
        x_new = x_rand;
    } else {
        float m = (x_rand[1] - x_nearest[1]) / (x_rand[0] - x_nearest[0]);

        x_new.push_back((sign(x_rand[0] - x_nearest[0])) * (sqrt((pow(eta, 2)) / ((pow(m, 2)) + 1))) + x_nearest[0]);
        x_new.push_back(m * (x_new[0] - x_nearest[0]) + x_nearest[1]);

        if (x_rand[0] == x_nearest[0]) {
            x_new[0] = x_nearest[0];
            x_new[1] = x_nearest[1] + eta;
        }
    }
    return x_new;
}

// gridValue 函数，返回 "Xp" 位置的网格值
// 地图数据: 100 占用, -1 未知, 0 空闲
int gridValue(nav_msgs::OccupancyGrid &mapData, std::vector<float> Xp) {
    float resolution = mapData.info.resolution;
    float Xstartx = mapData.info.origin.position.x;
    float Xstarty = mapData.info.origin.position.y;

    float width = mapData.info.width;
    std::vector<signed char> Data = mapData.data;

    float indx = (floor((Xp[1] - Xstarty) / resolution) * width) + (floor((Xp[0] - Xstartx) / resolution));
    return Data[int(indx)];
}

// ObstacleFree 函数，检查路径是否无障碍
char ObstacleFree(std::vector<float> xnear, std::vector<float> &xnew, nav_msgs::OccupancyGrid mapsub) {
    float rez = float(mapsub.info.resolution) * 0.2;
    float stepz = int(ceil(Norm(xnew, xnear) / rez));
    std::vector<float> xi = xnear;
    char obs = 0;
    char unk = 0;

    for (int c = 0; c < stepz; c++) {
        xi = Steer(xi, xnew, rez);

        if (gridValue(mapsub, xi) == 100) {
            obs = 1;
        }

        if (gridValue(mapsub, xi) == -1) {
            unk = 1;
            break;
        }
    }

    char out = 0;
    xnew = xi;
    if (unk == 1) {
        out = -1;
    } else if (obs == 1) {
        out = 0;
    } else {
        out = 1;
    }

    return out;
}
