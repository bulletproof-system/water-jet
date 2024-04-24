## 使用方法
1. 单模块调试：
   1. ``roslaunch navigation nav_test.launch``，启动simple_goal和move_base
   2. ``rosrun navigation target.py``，用于向导航节点发航点和急停命令（cancel）
   3. 在终端中按提示输入指令:
      
2. 软件启动：
   ``roslaunch navigation nav.launch``

## navigate action api
```text
/navigation/action/Navigate.action
# 导航 action
actionlib_msgs/GoalID goal_id # 唯一标识
geometry_msgs/Pose pos # 导航位置, 世界坐标
---
string result # 结果 'success' | `fail` | `cancel`
---
uint8 percentage # 进度百分数
string cur_state # 'normal' | 'barrier'
```


 
        
   
