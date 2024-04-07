## 使用方法
1. 单模块调试：
   1. ``roslaunch nav_pkg nav_test.launch``
   2. ``rosrun nav_pkg usr_nav_goal_publisher.py``
   3. 在终端中输入航点坐标
2. 软件启动：
   ``roslaunch nav_pkg nav.launch``

## nav_pkg api接口
1. nav_pkg包含``simple_goal``节点。
   - 它订阅了``/usr_nav_goal``话题，消息类型为String。
    主控可以向该话题发送航点坐标（String类型的消息，如``"4.0 2.0 0.0"``）。之后机器人就会导航到航点。在这个过程中动态避障。
   - 它发布``/usr_nav_result``话题，消息类型为String。
     主控可以订阅该话题以判断导航是否完成。导航完成时会发送String类型消息，消息内容为``"Finish"`` 
 
        
   
