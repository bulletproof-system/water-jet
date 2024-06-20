# navigation包说明
## 功能
- 导航
- 动态避障
- 导航进度反馈

## 效果演示
<table>
  <tr>
    <td><img src="docs/动态避障.gif" alt="Output GIF" style="height: 300px;"></td>
    <td><img src="docs/动态避障_仿真.gif" alt="Flower Pot" style="height: 300px;"></td>
  </tr>
</table>

## 使用方法
1. 导航模块调试：
   1. ``roslaunch navigation nav_test.launch``，启动simple_goal和move_base
   2. ``rosrun navigation target.py``，用于模拟主控向导航节点发送航点和取消
   3. 在终端中按提示输入指令
      
2. 软件启动：
   ``roslaunch navigation nav.launch``

## navigate action api
```text
/navigation/action/Navigate.action
# 导航 action api
geometry_msgs/Pose pos # 导航位置, 世界坐标
---
string result # 结果 'success' | 'fail' | 'cancel' | 'error'
---
uint8 percentage # 进度百分数(0~100)
string cur_state # 'normal' | 'barrier' | 'planning'
```
### Action Subscribed Topics

- `/navigation/navigate/cancel`
  - 取消导航

- `/navigation/navigate/goal`
  - 开始导航

### Action Published Topics

- `/navigation/navigate/status`
  - goal 状态

- `/navigation/navigate/feedback `
  - 导航进度
  
- `/navigation/navigate/result`
  - 导航结果




 
        
   
