# water-jet ROS 端

## 📖 介绍

water-jet ROS 端项目, 根据功能划分为多个包。

## 💿 运行

```bash
# 启动 ROS
roslaunch controller use_case_1.launch # 真实环境
roslaunch controller use_case_1_sim.launch # 仿真环境
```

## 🗂️ 文件说明

```none
ros
├── build
├── devel
└── src
│   ├── controller # 控制器
│   ├── map_provider # 地图
│   ├── navigation # 导航
│   ├── object_detect # 目标检测
│   ├── pot_database # 数据库
│   ├── robot_arm # 机械臂
│   ├── yolo_detector # YOLO 模型
│   ├── tf2_web_publisher # 前端通信相关
│   ├── world_simulation # 仿真环境
│   ├── wpb_home 
│   └── wpr_simulation
└── README.md
```
