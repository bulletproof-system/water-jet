#!/home/yanhaojun/miniconda3/envs/yolo/bin/python
# -*- coding: utf-8 -*-

import os
import cv2
import rospy
import numpy as np
from ultralytics import YOLO

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolo_detector.msg import BoundingBox, BoundingBoxes

cur_dir = os.path.dirname(os.path.abspath(__file__))
pkg_dir = os.path.dirname(cur_dir)
weights_dir = os.path.join(pkg_dir, 'weights')

class YoloDetector:
    def __init__(self):
        # 加载参数
        weight_path = rospy.get_param('~weight_path', os.path.join(weights_dir, 'yolov8n.pt'))
        image_topic = rospy.get_param('~image_topic', '/image_topic')
        publish_topic = rospy.get_param('~publish_topic', '/yolo_detector/BoundingBoxes')
        self.camera_frame = rospy.get_param('~camera_frame', '')
        model_conf = rospy.get_param('~conf', '0.3')                    # 模型置信度阈值
        self.visualizable = rospy.get_param('~visualizable', 'True')    # 是否可视化
        self.publish_rate = rospy.get_param('~publish_rate', 2)         # 默认每秒2次

        # device: cpu
        self.device = 'cpu'
        # self.model = YOLO(weight_path, task='detect')
        # self.model.fuse()               # 混合 BN 与相邻的卷积层，加快推理
        # self.model.conf = model_conf
        self.model = None
        self._load_model(weight_path, model_conf)

        self.color_image = Image()
        self.get_image_status = False
        self.rate = rospy.Rate(self.publish_rate)

        # 订阅图片
        self.color_sub = rospy.Subscriber(
            image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=52428800,
        )

        # 发布目标位置（BoundingBoxes.msg）
        self.position_pub = rospy.Publisher(
            publish_topic,
            BoundingBoxes,
            queue_size=1
        )

        # 发布检测结果图片
        self.image_pub = rospy.Publisher(
            '/yolo_detector/detection_image',
            Image,
            queue_size=1
        )

        while not self.get_image_status:
            rospy.loginfo("[yolo_detector] Waiting for image.")
            rospy.sleep(1)

    def _load_model(self, weight_path, model_conf):
        weight_ext = os.path.splitext(weight_path)[1]

        if weight_ext == '.pt':
            self.model = YOLO(weight_path, task='detect')
            self.model.fuse()  # 混合 BN 与相邻的卷积层，加快推理
        elif weight_ext == '.onnx':
            if not os.path.exists(weight_path):
                pt_path = weight_path.replace('.onnx', '.pt')
                if os.path.exists(pt_path):
                    self.model = YOLO(pt_path, task='detect')
                    self.model.export(format='onnx')
                else:
                    raise FileNotFoundError(f"Neither {weight_path} nor {pt_path} exists.")
            self.model = YOLO(weight_path, task='detect')
        elif 'openvino' in weight_path:
            if not os.path.exists(weight_path):
                pt_path = weight_path.replace('_openvino_model', '.pt')
                if os.path.exists(pt_path):
                    self.model = YOLO(pt_path, task='detect')
                    self.model.export(format='openvino')
                else:
                    raise FileNotFoundError(f"Neither {weight_path} nor {pt_path} exists.")
            self.model = YOLO(weight_path, task='detect')
        else:
            raise ValueError(f"Unsupported weight file extension: {weight_ext}")

        self.model.conf = model_conf

    def image_callback(self, image: BoundingBoxes):
        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header
        self.boundingBoxes.image_header = image.header
        self.get_image_status = True

        # 将ROS图像消息转换为NumPy数组并进行颜色转换，从而给 yolo 进行检测
        self.color_image = np.frombuffer(
            image.data,
            dtype=np.uint8
        ).reshape(image.height, image.width, -1)

        # Check the number of channels
        if self.color_image.shape[2] == 1:
            # Single channel (grayscale) image
            self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_GRAY2RGB)
        elif self.color_image.shape[2] == 2:
            # Two channels, handle appropriately, for example, using only one channel
            self.color_image = cv2.cvtColor(self.color_image[:, :, 0], cv2.COLOR_GRAY2RGB)
        elif self.color_image.shape[2] == 3:
            # Standard BGR image
            self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)
        else:
            raise ValueError(f"Unexpected number of channels: {self.color_image.shape[2]}")

        # 检测
        results = self.model(self.color_image, show=False, conf=0.3)

        # 显示和发布结果
        self.show_detection(results, image.height, image.width)
        cv2.waitKey(3)

        # 控制发送速率
        self.rate.sleep()

    # 显示结果
    def show_detection(self, results, height, width):
        # 绘制检测结果框
        self.frame = results[0].plot()
        print(results[0].speed['inference'], flush=True)

        # 计算并显示FPS
        fps = 1000.0/ results[0].speed['inference']
        cv2.putText(
            self.frame,
            f'FPS: {int(fps)}',
            (20, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA
        )

        # 处理每个检测框并发布
        for result in results[0].boxes:
            boundingBox = BoundingBox()
            boundingBox.xmin = np.int64(result.xyxy[0][0].item())
            boundingBox.ymin = np.int64(result.xyxy[0][1].item())
            boundingBox.xmax = np.int64(result.xyxy[0][2].item())
            boundingBox.ymax = np.int64(result.xyxy[0][3].item())
            boundingBox.Class = results[0].names[result.cls.item()]
            boundingBox.probability = result.conf.item()
            self.boundingBoxes.bounding_boxes.append(boundingBox)
        self.position_pub.publish(self.boundingBoxes)
        self.publish_image(self.frame, height, width)

        # 是否可视化检测结果
        if self.visualizable :
            cv2.imshow('yolo_detector', self.frame)

    # 将NumPy数组图像转换为ROS图像消息并发布
    def publish_image(self, imgdata, height, width):
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = self.camera_frame
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'
        image_temp.data = np.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3
        self.image_pub.publish(image_temp)

def main():
    rospy.init_node('yolo_detector', anonymous=True)
    yolo_detector = YoloDetector()
    rospy.spin()

if __name__ == "__main__":
    main()
