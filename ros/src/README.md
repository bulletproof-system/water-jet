## 浇花节点

### 功能：

- 控制机械臂抬升到花盆高度
- 控制水泵通电，为花盆浇花（目前用语音输出替代）

### 调试方法

- action topic："/aim"
- 消息格式：见msg
  - AimGoal.msg：uint32 id
  - AimResult.msg：bool success & string info
- **单**模块调试：`roslaunch robot_arm arm.launch`，模拟一个client调用action