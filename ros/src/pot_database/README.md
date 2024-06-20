# pot database包说明
## 功能

- 保存花盆信息列表

```
# database/msg/PotInfo.msg
# 单个花信息

uint32 id # 花盆 id
geometry_msgs/Pose pose # 世界坐标
uint8[] data # pcd 文件
uint8[] picture # 花照片
bool active # 是否自动浇灌
string last_water_date # 上次浇水时间
```
## Published Topics



## Services
- `/database/pot/list`

  返回所有花盆详细信息

  ```
  # /database/srv/GetPotList.srv
  # 花盆信息列表
  
  ---
  PotInfo[] pots
  ```

- `/database/pot/set`

  设置花盆信息

  ```
  # /database/srv/SetPotInfo.srv
  
  PotInfo info
  ---
  bool success
  ```
- `/database/pot/set_active`

  设置花盆 active

  ```
  # /database/srv/SetPotActive.srv
  
  uint32 id
  bool active
  ---
  bool success
  ```

- `/database/pot/get`

  获取花盆信息

  ```
  # /database/srv/GetPotInfo.srv
  
  uint32 id
  ---
  bool success
  PotInfo info
  ```

- `/database/pot/delete`

  删除指定花盆

  ```
  # /database/srv/DeletePot.srv
  
  uint32 id
  ---
  bool success
  ```

### Published Topics

- `/database/pot/update`

  更新/删除的 pot id 列表

  ```
  # /database/msg/PotUpdate.msg
  
  uint32[] update_list
  uint32[] delete_list
  ```
