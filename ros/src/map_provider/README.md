# map_provider包说明
## 功能

- 实时建图
- 加载初始地图, 自动确定机器人位置
  - 以地图中心为世界坐标原点
- 保存地图(作为下一次启动的初始地图)
- 清空地图, 清楚保存的地图
- action
  - 自动建图
  - 手动建图

## Subscribed Topics

- `/map_provider/save_map`

  保存地图

  ```
  # map_provider/msg/SaveMap.msg
  
  string name # 值暂时没用
  ```

- `/map_provider/clear_map`

  清空地图

  ```
  # map_provider/msg/ClearMap.msg
  
  string name # 值暂时没用
  ```

  

- `/map_provider/set_position`

  设置机器人初始位置

  ```
  # map_provider/msg/SetPosition.msg
  
  geometry_msgs/Pose pos
  ```

## Published Topics

- `/map` 

  地图以机器人初始位置为中心

## Actions

```
# map_provider/action/InitMap.action
# 建图 action

string caller
---
string result # 结果 'success' | 'fail' | 'cancel' | 'error'
---
uint8 percentage # 进度百分数
```

### 自动建图

- type: `InitMap.action`

##### Action Subscribed Topics

- `/map_provider/auto_init_map/goal`
  - 发起建图
- `/map_provider/auto_init_map/cancel`
  - 停止建图

##### Action Published Topics

- `/map_provider/auto_init_map/status`
  - goal 状态

- `/map_provider/auto_init_map/feedback `
  - 建图进度
- /`map_provider/auto_init_map/result `
  - 建图结果

### 手动建图

- type: `InitMap.action`

#### Action Subscribed Topics

- `/map_provider/manual_init_map/goal`
  - 发起建图
- `/map_provider/manual_init_map/cancel`
  - 停止建图

##### Action Published Topics

- `/map_provider/manual_init_map/status`
  - goal 状态

- `/map_provider/manual_init_map/feedback `
  - 建图进度
- `/map_provider/manual_init_map/result `
  - 建图结果
