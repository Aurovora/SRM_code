模块概述
coord模块主要用于坐标系之间的转换和管理。它定义了多种常用的坐标表示方式（旋转矩阵、欧拉角、直角坐标、球坐标、齐次变换矩阵）及其相互转换的函数，并提供了一个Solver类来管理和执行在机器人或视觉系统中常见的几个关键坐标系（世界、IMU/陀螺仪、相机、枪口）之间的转换。
1.base：实现单个坐标表示形式之间的基础数学转换
2.solver：利用base提供的基础转换，结合系统配置参数，来管理和计算不同物理坐标系之间的复杂变换。

1.base
欧拉角 $\leftrightarrow$ 旋转矩阵
功能：
EAngleToRMat：实现了 $(\text{yaw}, \text{pitch}, \text{roll})$ 欧拉角到旋转矩阵的转换。代码中是通过 $R_y(\text{yaw}) \cdot R_x(\text{pitch}) \cdot R_z(\text{roll})$ 的顺序进行矩阵乘法（$Y-X-Z$ 序列），这是一种特定的欧拉角约定。
RMatToEAngle：根据旋转矩阵的元素反解欧拉角，包含了对万向锁（Gimbal Lock, $R_{10}^2 + R_{11}^2 < 1\text{e-}8$）的特殊处理。
代码：
EAngle RMatToEAngle(RMat REF_IN rm) {
  if (rm(1, 0) * rm(1, 0) + rm(1, 1) * rm(1, 1) < 1e-8) {
    return {atan2(rm(0, 1), rm(2, 1)), asin(-rm(1, 2)), 0};
  } else {
    return {atan2(rm(0, 2), rm(2, 2)), asin(-rm(1, 2)), atan2(rm(1, 0), rm(1, 1))};
  }
}
RMat EAngleToRMat(EAngle REF_IN ea) {
  RMat rm_x, rm_y, rm_z;
  rm_x << 1, 0, 0, 0, cos(ea[1]), -sin(ea[1]), 0, sin(ea[1]), cos(ea[1]);
  rm_y << cos(ea[0]), 0, sin(ea[0]), 0, 1, 0, -sin(ea[0]), 0, cos(ea[0]);
  rm_z << cos(ea[2]), -sin(ea[2]), 0, sin(ea[2]), cos(ea[2]), 0, 0, 0, 1;
  return rm_y * rm_x * rm_z;
}

直角坐标 $\leftrightarrow$ 球坐标
功能：

代码：
CTVec STVecToCTVec(STVec REF_IN stv) {
  return {stv.z() * cos(stv.y()) * sin(stv.x()), -stv.z() * sin(stv.y()), stv.z() * cos(stv.y()) * cos(stv.x())};
}

STVec CTVecToSTVec(CTVec REF_IN ctv) {
  return {atan2(ctv.x(), ctv.z()), atan2(-ctv.y(), sqrt(ctv.x() * ctv.x() + ctv.z() * ctv.z())),
          sqrt(ctv.x() * ctv.x() + ctv.y() * ctv.y() + ctv.z() * ctv.z())};
}

旋转向量 $\leftrightarrow$ 旋转矩阵
功能：

代码：
RMat RVecToRMat(RVec REF_IN rv) {
  cv::Mat rv_cv, rm_cv;
  RMat rm;
  eigen2cv(rv, rv_cv);
  Rodrigues(rv_cv, rm_cv);
  cv2eigen(rm_cv, rm);
  return rm;
}
RVec RMatToRVec(RMat REF_IN rm) {
  cv::Mat rv_cv, rm_cv;
  RVec rv;
  eigen2cv(rm, rm_cv);
  Rodrigues(rm_cv, rv_cv);
  cv2eigen(rv_cv, rv);
  return rv;
}

2.solver


