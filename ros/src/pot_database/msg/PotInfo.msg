# database/msg/PotInfo.msg
# 单个花信息

uint32 id # 花盆 id
geometry_msgs/Pose pose # 世界坐标
uint8[] data # pcd 文件
uint8[] picture # 花照片
bool active # 是否自动浇灌
string last_water_date # 上次浇水时间