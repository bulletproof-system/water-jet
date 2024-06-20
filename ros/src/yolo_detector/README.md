# YOLO Detector(Water-Jet Only Look Once,WOLO)包说明
## 1. 功能优势

- 精准检测：我们基于 [YOLO](https://github.com/ultralytics/ultralytics) 实现了纯视觉定位方案，相比于原生包的点云方案能够更精准地检测目标
- 快速响应：使用原生 pytorch 能够在 70~300ms 之内检测目标，如果应用 ONNX 或 Openvino 可以达到至高 220% 的速度。
- 高扩展性：基于 YOLO 生态，可以很方便地嵌入不同的 YOLO 模型，并且支持自定义训练、微调模型



## 2. 如何使用

1. 拉取仓库：

    ```sh
    git clone https://github.com/Mxoder/WOLO.git
    
    # 将 WOLO/yolo_detector 作为一个功能包，放在自己项目的 `<your_workspace>/src` 下
    cp -r WOLO/yolo_detector <your_workspace>/src
    ```

2. 进入目录，安装 conda：

    ```sh
    cd WOLO
    chmod +x setup_miniconda.sh
    bash setup_miniconda.sh
    ```

3. 重启终端或新开一个终端，新建虚拟环境：

    ```sh
    # 创建 yolo 环境 (python 3.10)
    conda create -n yolo python=3.10 -y
    
    # 激活 yolo 环境
    source ~/miniconda3/bin/activate yolo
    
    # 在 yolo 环境下，如果是 3.10.x 就没问题了
    python --version
    # >>> 3.10.x
    ```

4. 重启终端或新开一个终端，安装依赖：

    ```sh
    # 激活 yolo 环境
    conda activate yolo
    
    # 安装 cpu 版本的 torch
    conda install pytorch==2.3.0 torchvision==0.18.0 torchaudio==2.3.0 cpuonly -c pytorch -y
    
    # 安装其余依赖
    pip install -r requirements.txt
    
    # 检测 torch 版本
    python -c "import torch;print(torch.__version__)"
    # >>> 2.3.0
    ```

5. 修改 `yolo_detector/launch/main.launch` 中的参数：

    ```
    weight_path: yolo 的权重路径
    image_topic: 订阅的图片话题，即需要检测的图片来源
    publish_topic: 发布的检测框话题，具体数据格式见 msg/BoundingBox.msg 和 msg/BoundingBoxes.msg
    conf: yolo 的检测信心阈值，即超过 conf 认为是检测到目标
    publish_rate: 检测发布速度，单位是张/秒，默认一张一秒
    ```

6. 此时已经可以正常使用功能包了：

    ```sh
    # 先在工作目录下 make
    cd <workspace> && catkin_make
    source devel/setup.sh
    
    # 方法一：单独跑检测器
    roslaunch yolo_detector main.launch
    
    # 方法二：将检测器作为 node 加入项目的主 launch 文件中
    # 在主 launch 中加一行 <include file="$(find yolo_detector)/launch/main.launch"/>
    ```

    

