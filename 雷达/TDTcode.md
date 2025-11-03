# TDT雷达模块理解
## what & why
雷达模块是雷达站上运行的算法系统，拥有固定的、较高的视角，可为战队提供全图的透视效果，获取实时、可靠的敌方机器人位置、身份和运动轨迹信息，从而辅助战术决策和精准打击 。
## 实战流程
由3个步骤组成
### lidar：实现了将动态点云转换为质心目标的聚类过程
+ 点云：它由激光雷达（LiDAR）传感器发射激光束，测量激光触及物体后返回的时间和角度，从而计算出每个点的三维坐标 $(x, y, z)$。每个点都代表传感器视野范围内物体表面的一个采样点。
+ 质心：提取质心是将复杂的点云数据简化和抽象，用一个单一的、稳定的点来代表目标的位置。（也方便后续的滤波）
~~看到这我其实有个问题：点云中应该包含了很多物体，如何区分不同物体，并分别提取出质心？由此引入了聚类↓~~
+ 聚类：将属于同一个物理物体的点云划分到一个数据组中。TDT采用的是欧几里得聚类（适用于形状规则且分离度高（有明显间距）的目标定位。且在赛场上物体数量动态变化，因此不适用需要预设目标数量的算法。
`Cluster::Cluster()`
```mermaid
graph TD
    A[Start: ROS 2 启动 Cluster 节点] --> B{Cluster::Cluster() 构造函数被调用};

    subgraph 初始化与配置 (仅执行一次)
        B --> C[设置节点身份: Node("cluster_node")];
        C --> D[日志记录: RCLCPP_INFO("cluster_node start")];
        D --> E{创建订阅者 (Subscriber)};
        E --> F[话题: /livox/lidar_dynamic];
        F --> G[消息类型: sensor_msgs::msg::PointCloud2];
        G --> H[绑定回调函数: Cluster::callback];
        H --> I{创建发布者 (Publisher)};
        I --> J[话题: /livox/lidar_cluster];
        J --> K[消息类型: sensor_msgs::msg::PointCloud2];
    end

    K --> L[Finish: 节点初始化完成];
    L --> M[等待数据: 节点进入 ROS 2 事件循环];

    subgraph 持续运行 (事件驱动)
        M --> N(数据到达: /livox/lidar_dynamic 收到新消息);
        N --> O[自动触发: Cluster::callback 函数被执行];
        O --> P[执行聚类和质心计算];
        P --> Q[发布结果: /livox/lidar_cluster];
```
