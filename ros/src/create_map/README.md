# 建图 v0.1.0 说明

> 包名：create_map
>
> 目前是由官方包 `wpr_home_behaviors` 改过来的，只缩减了不必要的重复 build，后续再考虑找到最小依赖

[TOC]



## 主启动

由 `create_map/launch/main.launch` 启动，`main.launch` 内容如下：

```xml
<launch>
    <!-- 建图节点，调用 scripts/map_creator 中的 MapCreator 类 -->
    <node name="create_map_node" pkg="create_map" type="map_creator" output="screen">
    </node>

    <!-- 目标检测节点，调用 src/create_map_objects_3d.cpp -->
    <node pkg="create_map" type="create_map_objects_3d" name="create_map_objects_3d" output="screen">
        <param name="topic" type="string" value="/kinect2/sd/points"/>
        <param name="start" type="bool" value="true"/>
    </node>
</launch>
```



### 建图

在 `scripts/map_creator` 中，由 `MapCreator` 类来负责建图，核心内容如下：

```python
class MapCreator:
    def __init__(self):
        rospy.Service('/control/create_map/start', Base, self.start)    # 启动建图路由
        rospy.Service('/control/create_map/save', Base, self.save)      # 保存地图路由

        self.process = None                                             # 保存进程对象
    
    def start(self, req):

        if self.process != None:                                        # 防止重复建图
            return BaseResponse("Map Creator is Running")
        
        def target():
            if rospy.get_param("simulate"):
                os.system("roslaunch create_map create_map_sim.launch")
            else:
                # TO BE DONE
                os.system("roslaunch create_map create_map_real.launch")
        
        self.process = multiprocessing.Process(target=target)           # 启动建图进程
        self.process.start()        

        return BaseResponse("map creating success")
    
    def save(self, req):
        map_name = req.request                                          # 从主控获取map_name
        os.system('rosrun map_server map_saver -f ' + map_name)         # map_server保存地图

        if self.process is not None:                                    # 终止建图进程
            self.process.terminate()
            self.process = None
        
        return BaseResponse("map saving success")
```

目前逻辑：

- 建图根据 gmapping 的 launch 改过来，由 start 路由管控，会调用 `create_map/launch/create_map_sim.launch` 来开启建图（即开启激光雷达扫图）
- 保存图片仍采用传统的 map_server 存下 pgm 文件



### 目标检测

主要实现逻辑在 `create_map/src/create_map_objects_3d.cpp` 中，改自 `wpb_home_behaviors` 中的目标检测。

当主启动 `main.launch` 后，目标检测节点会持续检测雷达范围内是否有目标物体，并以 Publisher 的形式、由近到远返回目标物体的中心点坐标（三维，有 x，y，z）。如果需要修改为边界坐标，可以在 `create_map_objects_3d.cpp` 的 323 行开始找到一段新增修改逻辑，目前是计算了 `center[XYZ]`，可以修改成 publish `boxMarker.[xyz][(Max)(Min)]`。

```cpp
// new: center
geometry_msgs::PointStamped center_point;
center_point.header.stamp = ros::Time::now();
center_point.header.frame_id = "base_footprint";

// calculate center point
float centerX = (boxMarker.xMax + boxMarker.xMin) / 2.0;
float centerY = (boxMarker.yMax + boxMarker.yMin) / 2.0;
float centerZ = (boxMarker.zMax + boxMarker.zMin) / 2.0;

center_point.point.x = centerX;
center_point.point.y = centerY;
center_point.point.z = centerZ;

obj_center_pub.publish(center_point);
```

