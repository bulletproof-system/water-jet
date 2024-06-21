#!/bin/bash

# 检查是否已经安装了 Miniconda
if [ -d "$HOME/miniconda3" ]; then
    echo "Miniconda 已经安装在 $HOME/miniconda3"
else
    # 创建 miniconda 目录
    mkdir -p ~/miniconda3

    # 下载 miniconda 安装脚本
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh

    # 安装 miniconda
    bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3

    source ~/.bashrc

    # 初始化 conda
    ~/miniconda3/bin/conda init bash

    # 配置 conda
    conda config --set show_channel_urls yes
    conda config --set auto_activate_base false  # 取消自动激活 base 环境

    # 覆盖 ~/.condarc 文件
    cat <<EOL > ~/.condarc
channels:
  - defaults
show_channel_urls: true
default_channels:
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/r
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/msys2
custom_channels:
  conda-forge: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
  msys2: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
  bioconda: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
  menpo: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
  pytorch: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
  pytorch-lts: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
  simpleitk: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
  deepmodeling: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/
EOL

    # 清理原索引
    conda clean -i -y
fi

# 确保 conda 可用
source ~/miniconda3/bin/conda

# 检查是否已经存在 yolo 环境
if conda env list | grep -q "yolo"; then
    echo "yolo 环境已存在"
else
    # 创建 yolo 环境并安装 python 3.10
    conda create -n yolo python=3.10 -y
fi

# 激活 yolo 环境
source ~/miniconda3/bin/activate yolo

# 安装 pytorch 及相关包
conda install pytorch torchvision torchaudio cpuonly -c pytorch -y

# 配置 pip 源
mkdir -p $HOME/.config/pip
cat <<EOL > ~/.config/pip/pip.conf
[global]
index-url = https://pypi.tuna.tsinghua.edu.cn/simple
trusted-host = pypi.tuna.tsinghua.edu.cn
EOL

# 安装 python 包
pip install ultralytics rospkg openvino onnx onnxruntime==1.15.1
