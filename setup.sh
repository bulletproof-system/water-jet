#!/bin/bash
# 前端
# 检查是否安装了 nvm
if command -v nvm > /dev/null 2>&1; then
  echo "nvm 已安装"
else
  echo "nvm 未安装, 正在安装..."
  curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
  source ~/.bashrc
fi

# 检查是否安装了 node 17
if nvm ls 17 > /dev/null 2>&1; then
  echo "Node 17 已安装"
else
  echo "Node 17 未安装, 正在安装..."
  nvm install 17
fi

# 检查 corepack 是否启用
if pnpm -v > /dev/null 2>&1; then
  echo "pnpm 已启用"
else
  echo "pnpm 未启用, 正在启用..."
  corepack enable pnpm
fi

cd frontend 
pnpm install
pnpm build
cd ..

# ROS 端
# 获取当前目录
current_dir=$(pwd)
src_dir="$current_dir/ros/src"
workspace_dir="/home/robot/catkin_ws" # 部署目录
deploy_name="fri_g3_morning"


# 检查是否安装了 ROS
if command -v rosversion > /dev/null 2>&1; then
  echo "ROS 已安装"
else
  echo "ROS 未安装, 正在安装..."
  sudo apt-get install -y ros-melodic-desktop-full
  source /opt/ros/melodic/setup.bash
fi

# 检查是否安装了 ROS 依赖包

# 安装 ROS 依赖包
bash ./scripts/install_for_melodic.sh

# 安装 conda 
bash ./scripts/setup_miniconda.sh

# 部署 ROS 到指定目录
bash ./scripts/deploy_water_jet.sh ${src_dir} ${workspace_dir} ${deploy_name}

cd /home/robot/catkin_ws
catkin_make
cd ..
