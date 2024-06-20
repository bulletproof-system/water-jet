# object detect包说明
## 1.功能
object detect旨在使用YOLO进行物体检测和使用Kinect2相机获取深度信息来检测和跟踪花盆。该系统订阅多个ROS主题以同步图像、边界框和深度数据，处理这些数据以识别检测到的植物的3D坐标，并使用有关这些植物的信息更新数据库。它还提供了一项服务，以检查特定植物是否在一定距离阈值内被检测到。

## 2.Subscribers
- **`/cmd_vel`**: 订阅机器人的速度命令，以监控其当前的线速度和角速度。
- **`/yolo_detector/detection_image`**: 订阅来自YOLO检测器的检测到的物体图像。
- **`/yolo_detector/BoundingBoxes`**: 订阅来自YOLO检测器的边界框信息。
- **`/kinect2/hd/image_depth_rect`**: 订阅来自Kinect2相机的深度图像。
- **`/kinect2/hd/camera_info`**: 订阅来自Kinect2相机的相机内参。

## 3.Publishers
- **`/obj_centers`**: 以 `PointStamped` 格式发布检测到的花盆的3D坐标。

## 4.Service
- **`object_detect/check_pot`**: 检查给定ID的花盆是否在一定距离阈值内。

## 5.主要方法
- **`cmd_vel_callback`**: 更新机器人的当前线速度和角速度。
- **`camera_info_callback`**: 更新相机的内参。
- **`synced_callback`**: 处理同步的图像、边界框和深度数据，以检测和定位花盆。
- **`load_pots_from_database`**: 从数据库加载花盆的信息。
- **`handle_check_pot`**: 处理检查特定花盆是否被检测到的请求。
- **`handle_update_pots`**: 更新数据库中有关新检测到的花盆的信息。
