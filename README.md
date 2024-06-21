# Water-Jet
北京航空航天大学 BUAA 2024 春季 软件工程 嵌入式项目 软工 嵌入式 ROS Embedded SE Software Engineering

[![GitHub Repo stars](https://img.shields.io/github/stars/bulletproof-system/water-jet?style=social)](https://github.com/bulletproof-system/water-jet/stargazers)
[![license](https://img.shields.io/github/license/bulletproof-system/water-jet.svg)](https://github.com/bulletproof-system/water-jet/blob/main/LICENSE)
[![issue resolution](https://img.shields.io/github/issues-closed-raw/bulletproof-system/water-jet)](https://github.com/bulletproof-system/water-jet/issues)
[![open issues](https://img.shields.io/github/issues-raw/bulletproof-system/water-jet)](https://github.com/bulletproof-system/water-jet/issues)



## 🚀 部署

一键部署脚本
```bash
source deploy.sh
```

> [!warning]
>
> 可能需要再次运行 `scripts/setup_miniconda.sh` 

## 🖥 前端

```bash
# 启动前端
cd frontend
pnpm preview # deploy 后已经运行了 pnpm build
```

详细介绍见 [前端 README](frontend/README.md)

## 🤖 ROS

```bash
# 启动 ROS
roslaunch controller use_case_1.launch # 真实环境
roslaunch controller use_case_1_sim.launch # 仿真环境
```

详细介绍见 [ROS端 README](ros/README.md)

## 🏅 致谢

- [ultralytics/ultralytics](https://github.com/ultralytics/ultralytics)



## 🖊 引用

```bibtex
@misc{2024waterjet,
    title={Water-Jet},
    author={LTT, Mxode, Hao7un, Kun, Le},
    howpublished = {\url{https://github.com/bulletproof-system/water-jet}},
    year={2024}
}
```



## 📃 开源许可证

本项目采用 [GNU General Public License v3.0 开源许可证](https://github.com/bulletproof-system/water-jet/blob/main/LICENSE)。



## ⭐ Contributors

<a href="https://github.com/bulletproof-system/water-jet/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=bulletproof-system/water-jet" />
</a>