# coord函数实现autoaim逻辑
## 坐标转换部分重写
```cpp
//目标识别与筛选 //pnp解算 略
if (success) {
    // 将 PnP解算得到的平移向量tvec（相机坐标系）转换为CTVec类型
    coord::CTVec ctv_target;
    ctv_target << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
    //转换到球坐标系
    coord::STVec stv_target = coord::CTVecToSTVec(ctv_target);
    double relative_yaw = stv_target.x();//偏角
    double relative_pitch = stv_target.y();//仰角

    // EAngle（Yaw, Pitch, Roll）；rm_self_是Rmat类型，需要转化成eangle类型
    coord::EAngle ea_self = coord::RMatToEAngle(rm_self_); 
    double current_yaw = ea_self[0];
    double current_pitch = ea_self[1];
    //滤波 略
    
    //计算新的绝对目标角度
    new_yaw = current_yaw + relative_yaw;
    new_pitch = current_pitch + relative_pitch;
}
```