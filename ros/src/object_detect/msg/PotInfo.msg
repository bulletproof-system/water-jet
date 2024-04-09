# object_detect/msg/PotInfo.msg
# 单个花信息
uint32 id # 花盆 id
geometry_msgs/Pose pose # 世界坐标
sensor_msgs/PointCloud2 data # 点云类型数据
sensor_msgs/Image picture # 花照片
bool active # 是否自动浇灌