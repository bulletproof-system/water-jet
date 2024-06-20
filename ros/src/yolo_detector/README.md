# YOLO Detector(WOLO)包说明

>[!info]
>
>这是 yolo_detector 包，实际使用和 object_detect 联用，如果需要单独的目标识别包请访问：[WOLO](https://github.com/Mxoder/WOLO)

## 1. 功能优势

- 精准检测：我们基于 [YOLO](https://github.com/ultralytics/ultralytics) 实现了纯视觉定位方案，相比于原生包的点云方案能够更精准地检测目标
- 快速响应：使用原生 pytorch 能够在 70~300ms 之内检测目标，如果应用 ONNX 或 Openvino 可以达到至高 220% 的速度。
- 高扩展性：基于 YOLO 生态，可以很方便地嵌入不同的 YOLO 模型，并且支持自定义训练、微调模型



## 2. 如何使用

> [!warning]
>
> 目前环境部署已经集成至 [scrtipts/setup_miniconda.sh](../../../scripts/setup_miniconda.sh)，无需额外部署。
>
> 但使用时仍需要修改 [yolo_detector](scripts/yolo_detector_node.py) 的第一行的解释器路径，以匹配当前环境下的 yolo 环境解释器。具体来说，可以在激活 yolo 的 conda 环境情况下输入 `which python` 来找到解释器路径，一般只需要改动用户文件夹。

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



## 3. 参数详情

### 订阅者 (Subscribers)

1. **订阅速度**
    - 话题：`/cmd_vel`
    - 消息类型：`Twist`
    - 回调函数：`cmd_vel_callback`
    - 作用：订阅速度消息，更新当前的线速度和角速度。
2. **订阅 AMCL 位姿**
    - 话题：`amcl_pose`
    - 消息类型：`PoseWithCovarianceStamped`
    - 回调函数：`amcl_pose_callback`
    - 作用：订阅 AMCL 位姿消息，更新位置和协方差矩阵。
3. **订阅图像**
    - 话题：`image_topic`（从参数服务器获取）
    - 消息类型：`Image`
    - 回调函数：`image_callback`
    - 作用：订阅图像消息，将其转换为 NumPy 数组并进行 YOLO 检测。

### 发布者 (Publishers)

1. **发布目标位置**
    - 话题：`publish_topic`（从参数服务器获取）
    - 消息类型：`BoundingBoxes`
    - 作用：发布检测到的目标位置（边界框）。
2. **发布检测结果图像**
    - 话题：`/yolo_detector/detection_image`
    - 消息类型：`Image`
    - 作用：发布包含检测结果的图像。

### 参数 (Parameters)

1. **权重路径**
    - 参数名：`~weight_path`
    - 默认值：`os.path.join(weights_dir, 'yolov8n.pt')`
    - 可选值：`.../yolov8n.pt`、`.../yolov8n.onnx`、`.../yolov8n_openvino_model`
    - 作用：指定 YOLO 模型的权重文件路径。
2. **图像话题**
    - 参数名：`~image_topic`
    - 默认值：`/image_topic`
    - 作用：指定订阅的图像话题名称。
3. **发布话题**
    - 参数名：`~publish_topic`
    - 默认值：`/yolo_detector/BoundingBoxes`
    - 作用：指定发布边界框的目标话题名称。
4. **相机帧**
    - 参数名：`~camera_frame`
    - 默认值：`''`
    - 作用：指定相机帧的名称。
5. **模型置信度阈值**
    - 参数名：`~conf`
    - 默认值：`0.3`
    - 作用：指定 YOLO 模型的置信度阈值。
6. **是否可视化**
    - 参数名：`~visualizable`
    - 默认值：`True`
    - 作用：指定是否可视化检测结果。
7. **发布速率**
    - 参数名：`~publish_rate`
    - 默认值：`2`
    - 作用：指定发布检测结果的速率（每秒发布次数）。

### 回调函数

1. **cmd_vel_callback**
    - 作用：更新当前的线速度和角速度。
2. **amcl_pose_callback**
    - 作用：更新位置和协方差矩阵。
3. **image_callback**
    - 作用：处理接收到的图像消息，进行 YOLO 检测，并发布检测结果。

### 私有方法

1. **_load_model**
    - 作用：加载 YOLO 模型，支持 .pt, .onnx 和 OpenVino 格式的权重文件。
2. **show_detection**
    - 作用：显示和发布检测结果。
3. **publish_image**
    - 作用：将 NumPy 数组图像转换为 ROS 图像消息并发布。

### 主函数

- main
    - 作用：初始化 ROS 节点并启动 YoloDetector 实例。
